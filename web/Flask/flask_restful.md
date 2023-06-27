# flask-restful

[TOC]

```bash
pip install flask-restful
```

## Resource

## reqparse

## fields

## marshal

## api

```bash
from flask import Blueprint
from flask_restful import Api


blueprint = Blueprint('remote', __name__, url_prefix='/api/patients')
api = Api(blueprint)

```
