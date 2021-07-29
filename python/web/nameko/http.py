import json

from nameko.web.handlers import http

from .config import ENV


class HttpService(object):

    name = ENV.HTTPService.NAME

    @http('GET', '/get/<int:value>')
    def get(self, request, value):
        print(request.headers)
        return json.dumps({'value': value})
