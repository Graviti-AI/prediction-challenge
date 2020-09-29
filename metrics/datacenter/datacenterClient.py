# -*- coding: UTF-8 -*-
# !/bin/python
from datacenter.auth import Auth
from datacenter.contentSetClient import ContentSetClient

from datacenter.labelSetClient import LabelSetClient


class DataSet:

    def __init__(self, content_set_id, label_set_id, auth, content_store_url=None, label_store_url=None):
        self.content_set_id = content_set_id
        self.label_set_id = label_set_id
        self.auth = auth
        if auth is not None:
            self.content_set_client = ContentSetClient(auth.gatewayUrl + "content-store/", auth)
            self.label_set_client = LabelSetClient(auth.gatewayUrl + "label-store/", auth)
        else:
            self.content_set_client = ContentSetClient(content_store_url, auth)
            self.label_set_client = LabelSetClient(label_store_url, auth)

    def get_object(self, object_path):
        return self.content_set_client.get_object(self.content_set_id, object_path)

    def put_label(self, object_path, label_meta, label_values):
        return self.label_set_client.put_label(self.label_set_id, object_path, label_meta,
                                               label_values)


if __name__ == "__main__":
    auth = Auth("http://139.224.229.231:9002/gateway/", "user1008", "user1008")
    dataSet1 = DataSet("00000000-0000-0000-0000-000000000179",
                       "154e35ba-e895-4f09-969e-f8c9445effff",
                       auth)
    data = dataSet1.get_object("009610.jpg")
    print(data)
    data2 = dataSet1.get_object("not exist.jpg")
    print(data2)
    # dataSet1.putLabel("test1.jpg", {}, {})
    # dataSet1.putLabel("test1.jpg", {}, {})

    # dataset2 = DataSet("contentSetId2", "labelSetId2", auth)
