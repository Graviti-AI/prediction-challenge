# -*- coding: UTF-8 -*-
# !/bin/python
import json

from datacenter.client import Client
import time


class Auth:
    personal = 1  # 个人版
    corporate = 2  # 企业版

    def __init__(self, gateway_url, access_key):
        self.gatewayUrl = gateway_url
        if self.gatewayUrl.endswith("/") is False:
            self.gatewayUrl = self.gatewayUrl + "/"
        self.access_key = access_key

    def sign_headers(self, headers):
        if headers is None:
            headers = {}
        headers["X-Token"] = self.access_key
        return headers


# if __name__ == "__main__":
#     gatewayUrl = "http://demo-validation.graviti.cn/gateway/"
#     auth = Auth(gatewayUrl, "SongHong0", "SongHong0", 1)
#     test_headers = None
#     print(auth.sign_headers(test_headers))

if __name__ == "__main__":
    import logging

    from datacenter.auth import Auth
    from datacenter.client import read_data_from_file
    from datacenter.contentSetClient import ContentSetClient, get_or_create_test_content_set
    from datacenter.labelSetClient import LabelSetClient

    access_key = "Accesskey-6354ad862a2f1deb90a90"
    GATEWAY = "https://fat-gas.graviti.cn/gatewayv2/"

    logging.basicConfig(level=logging.DEBUG)

    AUTH = Auth(GATEWAY, access_key)
    client = ContentSetClient(AUTH.gatewayUrl + "content-store/", AUTH)

    # contents = client.list_content_sets()

    content_set_id = "23c8d8ae-33f1-4511-bec0-19a569236af5"
    # "/Users/renjie.hui/Desktop/testCase/big_file1.jpg"
    # "/Users/renjie.hui/Documents/1.mkv.zip"

    # client.direct_put_big_object(content_set_id, "", "big_file1.png", 900)

    client.put_object_from_file(content_set_id, "1.jpeg", "/Users/renjie.hui/Desktop/1.jpeg",
                                client.PUT_DIRECTLY, 10)

    # sensor_frames = {
    #     "CAM_BACK_LEFT112": {
    #         "objectPath": "tests/os1.png",
    #         "timestamp": 2345.1234,
    #         "filePath": "../tests/tests/os1.png"
    #     },
    #     "CAM_BACK_RIGHT112": {
    #         "objectPath": "README.md",
    #         "timestamp": 2345.5678,
    #         "filePath": "../README.md"
    #     }
    # }
    # client.direct_put_frame_objects(content_set_id, sensor_frames)
