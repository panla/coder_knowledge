from contextlib import ContextDecorator
from functools import wraps

from flask_sqlalchemy import SQLAlchemy, BaseQuery
from sqlalchemy import text

db = SQLAlchemy()


class BaseModel(db.Model):
    __abstract__ = True

    id = db.Column(db.Integer, primary_key=True)
    is_deleted = db.Column(db.Boolean, nullable=False, server_default=text('0'), comment="是否删除")
    created_time = db.Column(db.DateTime, nullable=False, server_default=text('CURRENT_TIMESTAMP'), comment='创建时间')
    updated_time = db.Column(
        db.DateTime, nullable=False, server_default=text('CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP'),
        comment='更新时间'
    )

    @property
    def create_time_(self):
        return self.create_time.strftime('%Y-%m-%d %H:%M:%S')

    @property
    def updated_time_(self):
        return self.updated_time.strftime('%Y-%m-%d %H:%M:%S')


class ModelOperate():

    @classmethod
    def get(cls, **kwargs):
        return cls.query.filter_by(**kwargs).first()

    @classmethod
    def list(cls, **kwargs) -> BaseQuery:
        return cls.query.filter_by(**kwargs)

    @staticmethod
    def rollback():
        db.session.rollback()

    @staticmethod
    def commit():
        db.session.commit()

    @staticmethod
    def flush():
        db.session.flush()

    @classmethod
    def add(cls, instance):
        """添加一个对象"""

        try:
            db.session.add(instance)
            cls.commit()
        except Exception as exc:
            logger.error(exc.__str__())
            cls.rollback()
        return instance

    @classmethod
    def add_flush(cls, instance):
        try:
            db.session.add(instance)
            cls.flush()
        except Exception as exc:
            logger.error(exc.__str__())
            cls.rollback()
        return instance

    def save(self):
        """保存一个实例"""

        try:
            db.session.add(self)
            self.commit()
        except Exception as exc:
            logger.error(exc.__str__())
            self.rollback()
        return self

    def save_flush(self):

        try:
            db.session.add(self)
            self.flush()
        except Exception as exc:
            logger.error(exc.__str__())
            self.rollback()
        return self

    def update(self, **kwargs):
        """更新字段的值

        调用 .save() 保存
        """

        for key, value in kwargs.items():
            if value != getattr(self, key):
                setattr(self, key, value)
        instance = self.save()
        return instance

    def update_flush(self, **kwargs):
        for key, value in kwargs.items():
            if value != getattr(self, key):
                setattr(self, key, value)
        instance = self.save_flush()
        return instance

    @classmethod
    def execute(cls, sql: str) -> bool:
        try:
            db.session.execute(sql)
            db.session.commit()
            return True
        except Exception as exc:
            logger.error(exc.__str__())
            cls.rollback()
            return False

    def bulk_insert_mappings(cls, mappings, return_defaults=False, render_nulls=False):
        """批量插入"""

        try:
            db.session.bulk_insert_mappings(
                cls, mappings=mappings, return_defaults=return_defaults, render_nulls=render_nulls
            )

            cls.commit()
            return True
        except Exception as exc:
            logger.error(exc.__str__())
            cls.rollback()
            return False

    def bulk_insert_mappings_flush(cls, mappings, return_defaults=False, render_nulls=False):
        """批量插入"""

        try:
            db.session.bulk_insert_mappings(
                cls, mappings=mappings, return_defaults=return_defaults, render_nulls=render_nulls
            )

            cls.flush()
            return True
        except Exception as exc:
            logger.error(exc.__str__())
            cls.rollback()
            return False

    @staticmethod
    def to_dict(instance, selects: tuple = None, excludes: tuple = None) -> dict:
        # 返回 dict 格式数据, 获取 instance 的各个字段以及属性

        if instance:
            if not hasattr(instance, '__table__'):
                raise Exception('<%r> does not have attribute for __table__' % instance)
            had_fields = instance.__table__.columns
            if selects:
                return {i: getattr(instance, i) for i in selects}
            elif excludes:
                return {i.name: getattr(instance, i.name) for i in had_fields if i.name not in excludes}
            else:
                return {i.name: getattr(instance, i.name) for i in had_fields}
        else:
            return {}

    @classmethod
    def to_representation(
            cls, instance, selects: tuple = None, excludes: tuple = None,
            second_attrs: tuple = None, children_attrs: tuple = None
    ) -> dict:

        info = cls.to_dict(instance, excludes, selects)

        if second_attrs:
            for attr in second_attrs:
                info.update({attr: cls.to_dict(getattr(instance, attr))})

        if children_attrs:
            for attr in children_attrs:
                children = getattr(instance, attr)
                for c in children:
                    info.setdefault(attr, []).append(cls.to_dict(c))

        return info


class Atomic(ContextDecorator):
    """
    Transaction context
    """

    def __init__(self, db, savepoint=None):
        self.db = db  # db
        self.savepoint = savepoint  # save point

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_value, traceback):

        if exc_type is None:
            try:
                self.db.session.commit()
            except Exception as e:
                logger.error(e.__str__())
                self.db.session.rollback()
        else:
            logger.error(f'exc_type: {exc_type}, exc_value: {exc_value}')
            self.db.session.rollback()


def atomic(using=None, savepoint=None):
    # Bare decorator: @atomic -- although the first argument is called
    # `using`, it's actually the function being decorated.
    if callable(using):
        return Atomic(db, savepoint)(using)
    # Decorator: @atomic(...) or context manager: with atomic(...): ...
    else:
        if not using:
            using = db
        return Atomic(using, savepoint)


def atomicDecorator(using=db):
    def atomic_decorator(func):
        @wraps(func)
        def wrapped(*args, **kwargs):
            try:
                with using.session.begin():
                    func(*args, **kwargs)
                using.session.commit()
            except Exception as e:
                using.session.rollback()

        return wrapped

    return atomic_decorator

