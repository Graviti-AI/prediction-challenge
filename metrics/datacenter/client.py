# -*- coding: UTF-8 -*-
# !/bin/python
import json
import logging

import requests
from requests import RequestException
from requests_toolbelt import MultipartEncoder

from . import logUtil


def read_data_from_file(file_path):
    if file_path is None:
        return None

    f = open(file_path, 'rb')
    data = f.read()
    f.close()
    return data


class Client:

    def __init__(self, url, auth=None):
        self.url = url
        self.auth = auth

    def upload_big_file(self, object_name, file_path, content_set_id, headers=None):
        try:
            if self.auth is not None:
                headers = self.auth.sign_headers(headers)
            data = {"filePath": object_name, "contentSetId": content_set_id}
            with open(file_path, "rb") as f:
                data["fileData"] = (object_name, f)
                m = MultipartEncoder(fields=data)
                headers['Content-Type'] = m.content_type
                response = requests.post(self.url + "putObject", data=m, headers=headers)
        except RequestException as e:
            logging.error("Failed to upload_big_file %s" % e)
            return False

        logging.debug(response.content)
        return True

    def send_post_json_body(self, method, post_body, headers=None):
        try:
            if self.auth is not None:
                headers = self.auth.sign_headers(headers)
            response = requests.post(self.url + method, post_body, headers=headers)
        except RequestException as e:
            logging.error("Failed to post %s\n%s\n%s" % (method, post_body, e))
            return False, None

        if response.ok is False:
            logging.error("%s failed" % method)
            logging.error(
                logUtil.print_http_request_and_response(response.request, response))
            return False, None

        content_type = response.headers["Content-Type"]
        if 'application/json' not in content_type:
            # For getURL, which should be modified to
            # return not 200 status if failed
            return True, response.content

        result = json.loads(response.text)
        if result["success"] is False:
            logging.error("%s failed" % method)
            logging.error(
                logUtil.print_http_request_and_response(response.request, response))
            return False, None

        logging.debug(
            logUtil.print_http_request_and_response(response.request, response))
        return True, result["data"]
