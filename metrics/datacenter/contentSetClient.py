# -*- coding: UTF-8 -*-
# !/bin/python
import json
import logging

from urllib3 import encode_multipart_formdata

from datacenter import client
from datacenter.auth import Auth
from datacenter.client import read_data_from_file
import requests
from requests import RequestException
from requests_toolbelt import MultipartEncoder
import time
import base64
import filetype
from oss2 import SizedFileAdapter, determine_part_size
from oss2.models import PartInfo
import oss2
import os
import uuid


class   ContentSetClient(client.Client):

    PUBLISHED = 7

    PUT_NORMALLY = 1
    PUT_DIRECTLY = 2

    CONTENT_SET = "contentSet"
    FRAME = "frame"


    direct_put_object_params = {}
    oss_security_token = {}

    def _get_put_permission(self, content_set_id, category, expired_in_sec, segment_name=None):
        params = self.direct_put_object_params.get(content_set_id, None)
        if params is None or params["expireAt"] < int(time.time()):
            post_data = json.dumps({
                "category": category,
                "id": content_set_id,
                "segmentName": segment_name,
                "expiredInSec": expired_in_sec,
            })
            _, params = self.send_post_json_body("getPutPermission", post_data)
            self.direct_put_object_params[content_set_id] = params
        return params

    # 本token有效事件范围为15min-1h
    def _get_security_token(self, content_set_id, expired_in_sec, segment_name=None):
        params = self.oss_security_token.get(content_set_id, None)
        if params is None or params["expireAt"] < int(time.time()):
            post_data = json.dumps({
                "expiredInSec": expired_in_sec,
                "contentSetId": content_set_id,
                "segmentName": segment_name,
            })
            _, params = self.send_post_json_body("getSecurityToken", post_data)
            self.oss_security_token[content_set_id] = params
        return params

    def _get_bucket_by_security_token(self, content_set_id, expired_in_sec, segment_name=None):
        params = self._get_security_token(content_set_id, expired_in_sec, segment_name)
        if params is None:
            return None, None, None
        if params["result"]["backendType"] != "oss":
            logging.error("direct_put_big_object error, unsupported backendType, backendType = %s" % params["result"][
                "backendType"])
            return None, None, None
        sts = oss2.StsAuth(params["result"]["accessKeyId"], params["result"]["accessKeySecret"],
                           params["result"]["token"])
        bucket = oss2.Bucket(sts, params["result"]["endpoint"], params["result"]["bucketName"])
        return bucket, params["result"]["prefix"], params["result"]["expireAt"]

    def direct_put_object(self, content_set_id, file_path, file_name, expired_in_sec=3600, segment_name=None):
        try:
            params = self._get_put_permission(content_set_id, self.CONTENT_SET, expired_in_sec, segment_name)
            if params is None:
                return False, None
            post_data = params["result"]
            post_data["key"] = params["extra"]["objectPrefix"] + file_name
            kind = filetype.guess(file_path)
            with open(file_path, 'rb') as f:
                if kind is None:
                    post_data["file"] = (file_name, f)
                else:
                    post_data["file"] = (file_name, f, kind.MIME)
                m = MultipartEncoder(post_data)
                response = requests.post(params["extra"]["host"], data=m, headers={"Content-Type": m.content_type})
        except RequestException as e:
            del self.direct_put_object_params[content_set_id]
            logging.error("direct_put_object error %s" % e)
            return False, None
        if response.status_code == 200:
            return True, response.content
        logging.error("direct_put_object failed %s" % response.content)
        del self.direct_put_object_params[content_set_id]
        return False, None

    # file_name为存储在oss中的完整路径，如c241dce2bae9be3179b896738114c84d/02908422-23d0-4b4d-9b5f-6b6d441d6364/avatar.png
    def list_multipart_upload_events(self, file_name, bucket):
        event_map = {}
        for upload_info in oss2.ObjectUploadIterator(bucket, file_name): # 列举指定Object分片上传事件
        # for upload_info in oss2.MultipartUploadIterator(bucket): # 列举bucket下分片上传事件
            event_map[upload_info.key] = upload_info.upload_id
        return event_map

    # file_name为文件名，如avatar.png
    def abort_multipart_upload_events(self, content_set_id, file_name=None, expired_in_sec=3600, segment_name=None):
        bucket, prefix, _ = self._get_bucket_by_security_token(content_set_id, expired_in_sec, segment_name)
        if bucket is None:
            return False
        file_name = prefix + file_name
        event_map = self.list_multipart_upload_events(file_name, bucket)
        for value in event_map.values():
            bucket.abort_multipart_upload(file_name, value)
        return True

    # file_name为存储在oss中的完整路径，如c241dce2bae9be3179b896738114c84d/02908422-23d0-4b4d-9b5f-6b6d441d6364/avatar.png
    def list_uploaded_parts(self, file_name, bucket):
        event_map = self.list_multipart_upload_events(file_name, bucket)
        if file_name not in event_map.keys():
            logging.info("list_uploaded_parts, there is no uploaded parts")
            return None, None
        part_number_map = {}
        for part_info in oss2.PartIterator(bucket, file_name, event_map[file_name]):
            part_number_map[part_info.part_number] = part_info.etag
        return event_map[file_name], part_number_map

    # file_name为文件名，如avatar.png
    def direct_put_big_object(self, content_set_id, file_path, file_name, expired_in_sec=3600, segment_name=None):
        try:
            bucket, prefix, expire_at = self._get_bucket_by_security_token(content_set_id, expired_in_sec, segment_name)
            if bucket is None:
                return False
            file_name = prefix + file_name

            total_size = os.path.getsize(file_path)
            # 最多分10000片，每片大小100KB-5GB
            part_size = oss2.determine_part_size(total_size, preferred_size=10 * 1024 * 1024)

            # 获取之前已上传部分及当时的upload_id，续传需保持与上一次上传时的part_size相同
            upload_id, uploaded_part_number_map = self.list_uploaded_parts(file_name, bucket)
            if upload_id is None:
                upload_id = bucket.init_multipart_upload(file_name).upload_id

            parts = []
            with open(file_path, 'rb') as f:
                part_number = 1
                offset = 0
                while offset < total_size:
                    num_to_upload = min(part_size, total_size - offset)
                    if uploaded_part_number_map is not None and part_number in uploaded_part_number_map:
                        parts.append(PartInfo(part_number, uploaded_part_number_map[part_number]))
                    else:
                        if int(time.time()) > expire_at:
                            bucket, _, expire_at = self._get_bucket_by_security_token(content_set_id, expired_in_sec, segment_name)
                            if bucket is None:
                                return False
                        result = bucket.upload_part(file_name, upload_id, part_number, SizedFileAdapter(f, num_to_upload))
                        if result.status != 200:
                            logging.error("upload_part error,result.status = %s" % result.status)
                            del self.oss_security_token[content_set_id]
                            return False
                        parts.append(PartInfo(part_number, result.etag))
                    offset += num_to_upload
                    part_number += 1
            complete_result = bucket.complete_multipart_upload(file_name, upload_id, parts)
        except RequestException as e:
            del self.oss_security_token[content_set_id]
            logging.error("direct_put_big_object error %s" % e)
            return False
        logging.info("direct_put_big_object done,complete_result.status = %s" % complete_result.status)
        if complete_result.status != 200:
            del self.oss_security_token[content_set_id]
            return False
        return True

    def direct_put_frame_objects(self, content_set_id, segment_name=None, sensor_frames=None, expired_in_sec=60):
        if sensor_frames is None or sensor_frames == {}:
            print()
            print("directPutFrameObjects error: sensor_frames is None or empty")
            print()
            return False
        frame_id = str(uuid.uuid4())

        try:
            for sensor_name, sensor_info in sensor_frames.items():
                params = self._get_put_permission(content_set_id, self.FRAME, expired_in_sec, segment_name)
                if params is None:
                    logging.error("direct_put_frame_object get put permission failed")
                    return False
                post_data = params["result"]

                if not str(sensor_info["objectPath"]).startswith("/"):
                    sensor_info["objectPath"] = "/" + sensor_info["objectPath"]
                post_data["key"] = params["extra"]["objectPrefix"] + sensor_name + sensor_info["objectPath"]
                post_data["x:incidental"] = base64.urlsafe_b64encode(
                    json.dumps({"timestamp": sensor_info["timestamp"], "sensorName": sensor_name,
                                "segmentName": segment_name, "frameId": frame_id,
                                "objectPath": sensor_name + sensor_info["objectPath"]}).encode()).decode()
                kind = filetype.guess(sensor_info["filePath"])
                with open(sensor_info["filePath"], 'rb') as f:
                    if kind is None:
                        post_data["file"] = (sensor_info["objectPath"], f)
                    else:
                        post_data["file"] = (sensor_info["objectPath"], f, kind.MIME)
                    m = MultipartEncoder(post_data)
                    response = requests.post(params["extra"]["host"], data=m, headers={"Content-Type": m.content_type})
                if str(response.content).find("OK") == -1:
                    logging.error("direct_put_frame_object failed %s,%s" % (response.content, sensor_info["objectPath"]))
                    del self.direct_put_object_params[content_set_id]
                    return False
                logging.info("direct_put_frame_object done")
        except RequestException as e:
            logging.error("direct_put_frame_object error %s" % e)
            del self.direct_put_object_params[content_set_id]
            return False

    """
      create_content_set
      input:
      "name": "test_name",                        contentSet名字
      "collected_at": "2019-10-18T15:04:05+08:00", 创建时间(选填)
      "collected_location": "上海五角场大学路",    创建地点(选填)
      "desc": "This is a test content set.",      contentSet描述(选填)
      "content_set_type": 0或1，   0普通，1 追踪类多传感器数据集

      return:
      if success:
          return "contentId created"

      if failed:
          return None
      """

    def create_content_set(
        self,
        name,
        content_set_type=0,
        collected_at="2019-10-18T15:04:05+08:00",
        collected_location="",
        desc=""):
        post_data = json.dumps({
            "collectedAt": collected_at,
            "collectedLocation": collected_location,
            "desc": desc,
            "name": name,
            "contentSetType": content_set_type
        })

        result, data = self.send_post_json_body("createContentSet", post_data)
        if result is False:
            return None
        return data["contentSetId"]

    """
      list_content_sets

      input: name eg: test

      return: success: listContentSets results: a list containing all the Content sets with user's owner_id example: [{
      "contentSetId":3998,"name":"test_1","desc":"This is a test content set.","createdAt":"2019-11-22T10:08:08Z",
      "collectedAt":"2019-10-18T07:04:05Z","collectedLocation":"上海五角场大学路"}] failed: None

      """

    def list_content_sets(self, name=None):
        post_data = json.dumps({
            "name": name
        })
        result, data = self.send_post_json_body("listContentSets", post_data)
        if result is False:
            return None
        return data["contentSets"]

    """
      update_content_set, current only support update name

      input: content_set_id eg: 1
             name eg: 'new content_set_name'

      return: success:  failed: None

      """

    def update_content_set(self, content_set_id, name, status):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "name": name,
            "status": status,
        })
        result, data = self.send_post_json_body("updateContentSet", post_data)
        return result

    """
          list_content_sets

          input: content_set_id eg: 12345

          return: success: listContentSets results: a list containing all the Content sets with user's owner_id example: [{
          "contentSetId":3998,"name":"test_1","desc":"This is a test content set.","createdAt":"2019-11-22T10:08:08Z",
          "collectedAt":"2019-10-18T07:04:05Z","collectedLocation":"上海五角场大学路"}] failed: None

          """

    def get_content_set(self, content_set_id):
        post_data = json.dumps({
            "contentSetId": content_set_id,
        })
        result, data = self.send_post_json_body("listContentSets", post_data)
        if result is False:
            return None
        content_sets = data["contentSets"]
        if len(content_sets) == 0:
            return None
        return data["contentSets"][0]

    """
      delete_content_sets

      input:
          Id and Name are optional
          Specify Id : delete ContentSet info with the specified Id
          Specify Name: delete ContentSet info with user's OwnerId and specified Name
          Or : delete all ContentSets with the specified owner_id
          id of contentSet eg: "12345678-1234-1234-123456789abc"

      return:
          success:
              return True
          failed:
              return False
      """

    def delete_content_sets(
            self,
            content_set_id=None,
            name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "name": name
        })
        result, data = self.send_post_json_body("deleteContentSets",
                                                post_data)
        return result

    """
      list_objects

      input:
          content_set_id:  id of contentSet
          segment_name: name of segment

      return:
          success:
              listObject results
          example:
              ["38275/iva7/../test/os1.png", "test2.jpg"]
          failed:
              None

      """

    def list_objects(self, content_set_id, segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name,
        })
        result, data = self.send_post_json_body("listObjects", post_data)
        if result is False:
            return None
        return data["objects"]

    """
      getObject

      input:
        "objectName": "content-store/test/os1.png",
        "contentSetId": 1,
        "segmentName": name of segment

      return:
          if success:
              return byte array of the object
          if failed:
              return None

      success example return byte array of the object:
      """

    def get_object(self, content_set_id, object_name, segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name,
            "filePath": object_name
        })
        result, data = self.send_post_json_body("getObject", post_data)
        if result is False:
            return None
        return data

    """
      putObject

      input:
        "contentSetId": 1     id of content set
        "objectName": "/test/os1.png",  上传文件在contentSet保存路径
        "localFile": "content-store/test/os1.png",  上传文件在本地的路径

      return:
          success:
              return True
          failed:
              return False
      """
    def put_object_from_file(self, content_set_id, object_name, local_file, put_method=PUT_DIRECTLY, expired_in_sec=60, segment_name=None):
        if put_method == self.PUT_NORMALLY:
            return self.put_object(content_set_id=content_set_id, object_name=object_name,
                                   data=read_data_from_file(local_file))
        return self.direct_put_object(content_set_id, local_file, object_name, expired_in_sec, segment_name)


    """
      putObject

      input:
        "contentSetId": 1     id of content set
        "objectName": "/test/os1.png",  上传文件在contentSet保存路径
        "Bytes": "12rfd7f",  上传文件的字节数组

      return:
          success:
              return True
          failed:
              return False
      """
    # No longer maintained
    def put_object(self, content_set_id, object_name, data):
        if data is None or data == "":
            print()
            print("putObject error: Bytes is None or empty")
            print()
            return False

        post_data = ({
            "contentSetId": str(content_set_id),
            "filePath": object_name,
            "fileData": (object_name, data)
        })

        encode_post_data = encode_multipart_formdata(post_data)
        headers = {"Content-Type": encode_post_data[1], "User-Id": "123"}
        post_data = encode_post_data[0]
        result, data = self.send_post_json_body("putObject", post_data,
                                                headers=headers)
        return result
        # try:
        #     response = requests.post(self.url + "putObject", data=data, headers=headers)
        # except RequestException as e:
        #     print("putObject Error: ", e)
        #     return False
        # else:
        #     printHTTPRequestAndResponse(response.request, response)
        #     return True

    """
      deleteObject

      input: content_set_id:  id of contentSet eg: 1
             objectLists:     object list you want to delete eg: [/test/os1.png]
             segment_name:    name of segment

      return:
          success:
              return True
          failed:
              return False
      """

    def delete_objects(self, content_set_id, object_list, segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name,
            "filePaths": object_list,
        })
        result, data = self.send_post_json_body("deleteObjects",
                                                post_data)
        return result

    """
      getUrl

      input:
          content_set_id :    id of contentSet eg: 1
          object_names:       the objectName in oss = content_set_name + fileName
          expired_in_sec:     expired in seconds [default: 60s]
          segment_name:       name of segment

      return:
          success:
              return url of the object
          failed:
              return None
      """

    def get_urls(self, content_set_id, object_names, expired_in_sec=60, segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name,
            "filePaths": object_names,
            "expiredInSec": expired_in_sec,
        })
        result, data = self.send_post_json_body("getUrls", post_data)
        if result is False:
            return None
        return data["urls"]

    def create_or_update_segment(self, content_set_id, name, desc=""):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "name": name,
            "desc": desc})
        result, _ = self.send_post_json_body("createOrUpdateSegment", post_data)
        return result

    def list_segments(self, content_set_id, names=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "names": names})
        result, _ = self.send_post_json_body("listSegments", post_data)

    def delete_segments(self, content_set_id, names, force_delete=False):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "names": names,
            "forceDelete": force_delete})
        result, _ = self.send_post_json_body("deleteSegments", post_data)

    """
      createOrUpdateSensor
      input:
          "contentSetId": "154e35ba-e895-4f09-969e-f8c9445efd2c",
          "name": "CAM_BACK_RIGHT",
          "spec": "TTX-009",
          "type": "camera"
          "desc": "The camera on left",
          "location": "上海五角场大学路",
          "extrinsicParams": {
              "rotation": {
                "w": 0.6924185592174665,
                "x": -0.7031619420114925,
                "y": -0.11648342771943819,
                "z": 0.11203317912370753
              },
              "translation": {
                "x": 1.03569100218,
                "y": 0.484795032713,
                "z": 1.59097014818
              }
          }
          "intrinsicParams": {
              "cameraMatrix": [
                [1256.7414812095406, 0, 792.1125740759628],
                [0, 1256.7414812095406, 492.7757465151356],
                [0, 0, 1]
              ],
              "distortionCoefficient": {
                "k1": 0.12,
                "k2": 0.33,
                "k3": 0.13,
                "k4": 0.93,
                "k5": 0.23,
                "p1": 1,
                "p2": 0.3
              }
          }

      return: true/false
      """

    def create_or_update_sensor(
            self,
            content_set_id,
            name,
            spec="",
            sensor_type="",
            desc="",
            location="",
            extrinsic_params=None,
            intrinsic_params=None,
            segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name,
            "name": name,
            "spec": spec,
            "type": sensor_type,
            "desc": desc,
            "location": location,
            "extrinsicParams": extrinsic_params,
            "intrinsicParams": intrinsic_params
        })

        result, _ = self.send_post_json_body("createOrUpdateSensor",
                                             post_data)
        return result

    """
      put_frame_objects

      input:
        "contentSetId": 1     id of content set
        "sensorFrames": {
          "sensorName1": {
            "objectPath":	"The path in remote",
            "timestamp":	1234.5678,
            "filePath": "The path of local file"
          },
          "sensorName2": {
            "objectPath":	"The path in remote",
            "timestamp":	1234.5678,
            "filePath": "The path of local file"
          }
        }

      return:
          success:
              return True
          failed:
              return False
      """
    # No longer maintained
    def put_frame_objects(self, content_set_id, segment_name=None, sensor_frames=None):
        if sensor_frames is None or sensor_frames == {}:
            print()
            print("putFrameObjects error: sensor_frames is None or empty")
            print()
            return False

        post_data = ({
            "contentSetId": str(content_set_id),
            "segmentName": segment_name
        })
        sensor_file_ifs = {}
        for sensorName, sensor_info in sensor_frames.items():
            sensor_file_ifs[sensorName] = {
                "objectPath": sensor_info["objectPath"],
                "timestamp": sensor_info["timestamp"],
            }
            post_data[sensorName] = (
                sensor_info["objectPath"],
                read_data_from_file(sensor_info["filePath"]))
        post_data["sensorFileInfos"] = json.dumps(sensor_file_ifs)

        body, content_type = encode_multipart_formdata(post_data)
        headers = {"Content-Type": content_type}
        result, data = self.send_post_json_body("putFrameObjects", body,
                                                headers=headers)
        return result

    """
      list_frames

      input: content_set_id eg: 1123123

      """

    def list_frames(self, content_set_id, segment_name=None):
        post_data = json.dumps({
            "contentSetId": content_set_id,
            "segmentName": segment_name
        })
        result, data = self.send_post_json_body("listFrames", post_data)
        if result is False:
            return None
        return data["frames"]


def get_or_create_test_content_set(content_set_client, owner_id, group_id,
    content_set_name):
    content_sets = content_set_client.list_content_sets(owner_id, group_id,
                                                        content_set_name)
    if content_sets is not None and len(content_sets) > 0:
        return content_sets[0]["contentSetResp"]["contentSetId"]

    return content_set_client.create_content_set(owner_id, content_set_name,
                                                 group_id, 1)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    test_owner_id = "user1008"
    test_owner_password = "user1008"
    auth = Auth("https://dev-labeling.graviti.cn/gateway/", test_owner_id,
                test_owner_password)
    dev_client = ContentSetClient(auth.gatewayUrl + "content-store/", auth)

    test_content_set_id = get_or_create_test_content_set(dev_client,
                                                         auth.owner_id,
                                                         auth.group_id,
                                                         "Python Demo Tracking ContentSet")

    dev_client.create_or_update_sensor(test_content_set_id, "CAM_BACK_LEFT")
    dev_client.create_or_update_sensor(
        test_content_set_id,
        "CAM_BACK_RIGHT", "SPEC_AAA", "camera",
        "I am a description, i am rich",
        "I am a location",
        extrinsic_params={
            "translation": {"x": 1.03569100218, "y": 0.484795032713,
                            "z": 1.59097014818},
            "rotation": {
                "w": 0.6924185592174665,
                "x": -0.7031619420114925,
                "y": -0.11648342771943819,
                "z": 0.11203317912370753
            },
        },
        intrinsic_params={
            "cameraMatrix": [
                [1256.7414812095406, 0.0, 792.1125740759628],
                [0.0, 1256.7414812095406, 492.7757465151356],
                [0.0, 0.0, 1.0]
            ],
            "distortionCoefficient": {
                "p1": 1.1,
                "p2": 2.2,
                "k1": 3.3,
                "k2": 4.4,
                "k3": 5.5
            }
        })

    dev_client.put_frame_objects(test_content_set_id, {
        "CAM_BACK_LEFT": {
            "objectPath": "tests/os1.png",
            "timestamp": 1234.1234,
            "filePath": "../tests/tests/os1.png"
        },
        "CAM_BACK_RIGHT": {
            "objectPath": "README.md",
            "timestamp": 1234.5678,
            "filePath": "../README.md"
        }
    })

    dev_client.put_frame_objects(test_content_set_id, {
        "CAM_BACK_LEFT": {
            "objectPath": "tests/os2.png",
            "timestamp": 2345.1234,
            "filePath": "../tests/tests/os1.png"
        },
        "CAM_BACK_RIGHT": {
            "objectPath": "README2.md",
            "timestamp": 2345.5678,
            "filePath": "../README.md"
        }
    })

    dev_client.list_frames(test_content_set_id)

