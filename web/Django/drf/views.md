# views

[TOC]

## APIView

继承 `django.views.View`

### 属性和方法

| 属性或方法| 作用 | 备注 |
| :-: | :-: | :-: |
| renderer_classes | 渲染器 | rest_framework.renderers.* |
| parser_classes | 解析器 | rest_framework.parsers.* |
| authentication_classes | 验证器，列表 | 参考 rest_framework_jwt.authentication.JSONWebTokenAuthentication |
| throttle_classes | 频率控制 |  |
| permission_classes | 权限控制，列表 | 参考 rest_framework.permissions.BasePermission |
| content_negotiation_class | 内容协商? |  |

需要自行实现

- def get(self, request):
- def post(self, request):
- def put(self, request):
- def delete(self, request):

## generics

### GenericAPIView

继承 `rest_framework.views.APIView`

多了的属性和方法

| 属性或方法 | 作用 | 备注 |
| :-: | :-: | :-: |
| queryset | 获取所需的 QuerySet 数据 |  |
| get_queryset() | 获取所需的 QuerySet 数据 | 和 queryset 必选其中一个 |
| serializer_class | 序列化器 |  |
| get_serializer_class() | 获得序列化器 | 和 serializer_class 必选其中一个 |
| lookup_field | 根据此参数获得 Model 实例数据 | 默认 pk |
| get_object() | 获得 Model 实例数据 |  |
| pagination_class | 分页器 |  |

需要自行实现

- def get(self, request):
- def post(self, request):
- def put(self, request):
- def delete(self, request):

### 其他的 APIViews

继承 GenericAPIView 和 组合相对应的 Mixin

| generic 的 APIView | 所需要的 Mixin | HTTP function | Mixin 提供的可重写 function |
| :-: | :-: | :-: | :-: |
| ListAPIView | ListModelMixin| get | list |
| CreateAPIView | CreateModelMixin | post | create |
| RetrieveAPIView | RetrieveModelMixin | get | retrieve |
| UpdateAPIView | UpdateModelMixin | put patch | update partial_update |
| DestroyAPIView | DestroryModelMixin | delete | destory |

## viewsets

封装过度

### ViewSet

继承 `ViewSetMixin, APIView`

### GenericViewSet

继承 `ViewSetMixin, GenericAPIView`

### ModelViewSet

继承 `rest_framwork.mixins 的五个Mixin, GenericViewSet`

## 选择

逻辑简单的可以选择 generics

复杂的可以选择 APIView
