# 序列器

[toc]

## quick

```python
from reset_framework import serializers


class UserSerializer(serializers.ModelSerializer):
    class Meta:
        model = User
        fields = ['id', 'username', 'mobile', 'email', 'is_superuser', 'is_active']
        extra_kwargs = {'password': {'write_only': True}}
        # 密码不显示,创建用户时需要传递

  def validated_company_id(self, value):
    """校验某个字段"""
        company = Company.objects.filter(is_deleted='0', id=value).first()
        if not company:
            raise serializers.ValidationError('没有该公司id')
        return value

    def create(self, validated_data):
        """重写创建数据方法"""
        project = self.Meta.model.objects.create(**validated_data)
        return project

    def update(self, instance, validated_data):
        """重写更新方法"""
        for attr, value in validated_data.items():
            setattr(instance, attr, value)
        instance.save()
        return instance
```

## `to_representation`

改变序列化的输出，模型到Python

```python
from reset_framework import serializers


class UserSerializer(serializers.ModelSerializer):
    def to_representation(self, instance):
        data = super().to_representation(instance)
        data = data.update(view_count=instance.view_count + 1)
        return data
```

## `to_internal_value`

改变反序列化的输入，Python到模型

```python
from reset_framework import serializers


class UserSerializer(serializers.ModelSerializer):
    def to_internal_value(self, data):
        data = data.get('view_count') + 1
        return super().to_internal_value(data)
```
