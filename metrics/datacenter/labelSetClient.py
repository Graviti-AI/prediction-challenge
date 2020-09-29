# -*- coding: UTF-8 -*-
# !/bin/python
import json

from . import client


class LabelSetClient(client.Client):

    def create_label_set(self, meta, content_set_id, object_paths,
                         content_set_filter, version, label_set_type=None):
        if label_set_type is None:
            label_set_type = 3
        post_data = json.dumps({
            "meta": meta,
            "contentSetId": content_set_id,
            "contentSetFilter": content_set_filter,
            "objectPaths": object_paths,
            "type": label_set_type,
            "version": version
        })

        result, data = self.send_post_json_body("createLabelSet", post_data)
        if result is False:
            return None
        return data["labelSetId"]

    def get_label_set(self, label_set_id):
        post_data = json.dumps({
            "id": label_set_id
        })

        result, data = self.send_post_json_body("listLabelSets", post_data)
        if result is False or len(data["labelSets"]) == 0:
            return None
        # id is always unique
        return data["labelSets"][0]["labelSet"]

    def list_label_sets(self, content_set_id):
        post_data = json.dumps({
            "contentSetId": content_set_id
        })

        result, data = self.send_post_json_body("listLabelSets", post_data)
        if result is False or len(data["labelSets"]) == 0:
            return None
        return data["labelSets"]

    def delete_label_set(self, label_set_id):
        post_data = json.dumps({
            "labelSetId": label_set_id
        })
        result, data = self.send_post_json_body("deleteLabelSet", post_data)
        return result

    def update_label_set(self, label_set_id, meta):
        post_data = json.dumps({
            "labelSetId": label_set_id,
            "meta": meta
        })
        result, data = self.send_post_json_body("updateLabelSet", post_data)
        return result

    def list_labels(self, label_set_id, output_version=None, label_meta=None):
        post_data = json.dumps({
            "labelSetId": label_set_id,
            "outputVersion": output_version,
            "meta": label_meta
        })

        result, data = self.send_post_json_body("listLabels", post_data)
        if result is False:
            return None
        return data["labels"]

    def get_label(self, label_set_id, object_path):
        post_data = json.dumps({
            "labelSetId": label_set_id,
            "objectPath": object_path
        })

        result, data = self.send_post_json_body("getLabel", post_data)
        if result is False:
            return None
        return data["label"]

    def put_label(self, label_set_id, object_path, label_meta, label_values):
        post_data = json.dumps({
            "labelSetId": label_set_id,
            "objectPath": object_path,
            "labelMeta": label_meta,
            "labelValues": label_values
        })
        result, data = self.send_post_json_body("putLabel", post_data)
        return result

    def publish(self, label_set_id):
        post_data = json.dumps({
            "labelSetId": label_set_id,
            # published
            "status": 2,
        })
        result, data = self.send_post_json_body("updateLabelSetStatus",
                                                post_data)
        return result

    def list_object_paths(self, label_set_id) -> []:
        post_data = json.dumps({
            "labelSetId": label_set_id,
        })

        result, data = self.send_post_json_body("listObjectPaths", post_data)
        if result is False or len(data["objectPaths"]) == 0:
            return None
        return data["objectPaths"]

    def get_object(self, label_set_id, object_path, owner_id):
        post_data = json.dumps({
            "contentSetId": label_set_id,
            "filePath": object_path,
            "userId": owner_id,
        })

        result, data = self.send_post_json_body("getObject", post_data, headers={'User-Id': owner_id})
        if result is False:
            return None
        return data
