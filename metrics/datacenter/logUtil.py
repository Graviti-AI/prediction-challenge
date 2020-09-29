# -*- coding: UTF-8 -*-
# !/bin/python
from typing import Iterable


def print_http_request_and_response(request, response):
    lines = ["======================================================"]
    lines += print_http_request(request)
    lines.append("")
    lines += print_http_response(response)
    lines.append("======================================================")
    return "\n".join(lines)


def print_http_request(request):
    """
        putObject:
        ############## HTTP Request ##############
        POST http://47.103.90.26:9101/putObject
        User-Agent: python-requests/2.18.4
        Content-Length:  9818
        Accept-Encoding:  gzip, deflate
        Accept:  */*
        Connection:  keep-alive
        Content-Type:  multipart/form-data; boundary=96704080f7f2439281b9e8128757efee
        Body:  --96704080f7f2439281b9e8128757efee
        Content-Disposition: form-data; name="id"

        136
        --96704080f7f2439281b9e8128757efee
        Content-Disposition: form-data; name="filePath"
        ．．．

        其他方法:
        ############## HTTP Request ##############
        POST http://47.103.90.26:9101/listObjects
        User-Agent: python-requests/2.18.4
        Content-Length:  11
        Accept-Encoding:  gzip, deflate
        Accept:  */*
        Connection:  keep-alive

        {"id": 136}
        """
    lines = ["############## HTTP Request ##############", "%s %s" % (request.method, request.url)]
    lines += print_http_headers(request.headers)

    if request.body is None:
        return lines
    # 只有putObject的request.headers才有Content-Type信息
    # 所以用get方法访问dict,如果不存在Content-Type key字段,默认返回None
    content_type = request.headers.get("Content-Type", None)
    if content_type is not None and content_type.startswith(
            "multipart/form-data"):
        # e.g. multipart/form-data; boundary=831438b97f1574284afd11effd5c908d
        boundary = "--" + content_type[
                          content_type.find("boundary=") + len("boundary="):]
        groups = request.body.split((boundary + "\r\n").encode())
        for g in groups:
            if len(g) == 0:
                continue

            lines.append("")
            lines.append(boundary)
            if len(g) > 512:
                sections = g.split(b"\r\n")
                if len(sections) > 2:
                    lines.append(str(sections[0] + b"\r\n"))
                    lines.append(str(sections[1] + b"\r\n"))
                    lines.append("[%d bytes of object data]\r\n" % (
                            len(g) - len(sections[0]) - len(sections[1])))
                else:
                    lines.append("[%d bytes of object data]\r\n" % len(g))
            else:
                lines.append(str(g))
        # logging.debug("Body:", request.body.decode("unicode_escape",
        # errors="ignore"))

    else:
        # unicode -> 中文
        body = request.body.encode('utf-8').decode("unicode_escape")
        lines.append(body)
    return lines


def print_http_response(response):
    """
        case 1: output json

        ############## HTTP Response ##############
        Status-Code: 200 OK
        Date: Fri, 08 Nov 2019 02:08:34 GMT
        Content-Length: 43
        Content-Type: text/plain; charset=utf-8

        {"objects":["38275/iva7/../test/os1.png"]}

        case 2: output objects
        ############## HTTP Response ##############
        Status-Code: 200 OK
        Date: Fri, 08 Nov 2019 02:08:34 GMT
        Content-Length: 9448
        Content-Type: image/png

        [9448 bytes of object data]

        """
    lines = ["############## HTTP Response ##############",
             f'{response.status_code} {response.reason}']
    lines += print_http_headers(response.headers)
    lines.append("")
    if response.headers and "Content-Type" in response.headers:
        content_type = response.headers["Content-Type"]
        if content_type.startswith("text") or content_type.find("application/json") != -1:
            lines.append(response.text)
        else:
            lines.append("[%d bytes of object data]" % len(response.content))
    return lines


def print_http_headers(headers):
    lines = []
    for key in headers:
        lines.append("%s: %s" % (key, headers[key]))
    return lines
