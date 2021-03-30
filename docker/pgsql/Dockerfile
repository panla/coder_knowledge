FROM registry.cn-hangzhou.aliyuncs.com/panla/pg_13_1:v2

EXPOSE 5432

VOLUME ["/home/opt/pgsql", "/home/opt/pgsql/data", "/home/opt/pgsql/logs", "/home/opt/pgsql/archive", "/home/opt/pgsql/tmp"]

COPY ./conf.d /srv/conf.d

ENV TZ=Asia/Shanghai

ENV LC_ALL zh_CN.UTF-8

RUN chown postgres:postgres -R /srv/conf.d && su - postgres && initdb -D /opt/pgsql/data -E UTF8 --locale=zh_CN.utf8 && cp /srv/conf.d/postgresql.conf /opt/pgsql/data/postgresql.conf && cp /srv/conf.d/pg_hba.conf /opt/pgsql/data/pg_hba.conf && psql --command "create user rep1 replication login encrypted password 'rep1111';" && psql --command "alter role postgres with password 'password';"

CMD ["su - postgres", "-c", "pg_ctl -D /opt/pgsql/data restart"]
