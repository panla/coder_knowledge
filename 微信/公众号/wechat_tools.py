import aiohttp


class WeChatTool(object):
    def __init__(self, app_id, app_secret, state):
        self.app_id = app_id
        self.app_secret = app_secret
        self.state = state

    @staticmethod
    async def _request(method: str, url: str):
        async with aiohttp.ClientSession() as session:
            async with session.request(method=method, url=url) as response:
                data = await response.json()
                if 'errcode' in data:
                    raise Exception(f'{method} {url} error, data: {data}')
                return data

    @staticmethod
    async def request(method: str, url: str):
        for i in range(3):
            data = await self._request(method, url)
            if 'errcode' not in data:
                return data
            else:
                pass
        raise Exception(f'{method} {url} error, data: {data}')

    async def get_auth_code(self, redirect_uri: str, scope: str = 'snsapi_userinfo'):
        """获取 code

        :param redirect_uri: 配置的 redirect_uri
        :param scope: 获取用户信息模式
        :return: 重定向链接
        """

        api = 'https://open.weixin.qq.com/connect/oauth2/authorize'
        base_url = f'{self.app_id}&redirect_uri={redirect_uri}&response_type=code&scope={scope}&state={self.state}'
        url = f'{api}?appid={base_url}'
        data = await self.request('GET', url)
        code = data.get('code')
        return f'{redirect_uri}/?code={code}&state={self.state}'

    async def get_access_token(self, code: str):
        """获取 access_token

        :param code: 重定向过来的 code
        :return: {"access_token":"", "expires_in":7200, "refresh_token":"", "openid":"", "scope":""}
        """

        api = 'https://api.weixin.qq.com/sns/oauth2/access_token'
        base_url = f'{self.app_id}&secret={self.app_secret}&code={code}&grant_type=authorization_code'
        url = f'{api}?appid={base_url}'
        data = await self.request('GET', url)
        return data

    async def refresh_access_token(self, refresh_token):
        """刷新 access_token

        :param refresh_token: refresh_token
        :return: {"access_token":"", "expires_in":7200, "refresh_token":"", "openid":"", "scope":""}
        """

        domain = 'https://api.weixin.qq.com/sns/oauth2/refresh_token'
        url = f'{domain}?appid={self.app_id}&grant_type=refresh_token&refresh_token={refresh_token}'
        data = await self.request('GET', url)
        return data

    async def get_user_info(self, access_token, openid):
        """获取用户信息

        :param access_token: access_token
        :param openid: openid
        :return: {"openid":"", "nickname":"", "sex":1, "city":"", "country":"", "headimgurl":"", "privilege":""}
        """

        url = f'https://api.weixin.qq.com/sns/userinfo?access_token={access_token}&openid={openid}&lang=zh_CN'
        data = await self.request('GET', url)
        return data
