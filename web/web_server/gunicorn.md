# Gunicorn

## CMD

```text

gunicorn --help

usage: gunicorn [OPTIONS] [APP_MODULE]

optional arguments:
  -h, --help            show this help message and exit
  -v, --version         show program's version number and exit
  -c CONFIG, --config CONFIG
                        The Gunicorn config file. [None]
  -b ADDRESS, --bind ADDRESS
                        The socket to bind. [['127.0.0.1:8000']]
  --backlog INT         The maximum number of pending connections. [2048]
  -w INT, --workers INT
                        The number of worker processes for handling requests. [1]
  -k STRING, --worker-class STRING
                        The type of workers to use. [sync]
  --threads INT         The number of worker threads for handling requests. [1]
  --worker-connections INT
                        The maximum number of simultaneous clients. [1000]
  --max-requests INT    The maximum number of requests a worker will process before restarting. [0]
  --max-requests-jitter INT
                        The maximum jitter to add to the *max_requests* setting. [0]
  -t INT, --timeout INT
                        Workers silent for more than this many seconds are killed and restarted. [30]
  --graceful-timeout INT
                        Timeout for graceful workers restart. [30]
  --keep-alive INT      The number of seconds to wait for requests on a Keep-Alive connection. [2]
  --limit-request-line INT
                        The maximum size of HTTP request line in bytes. [4094]
  --limit-request-fields INT
                        Limit the number of HTTP headers fields in a request. [100]
  --limit-request-field_size INT
                        Limit the allowed size of an HTTP request header field. [8190]
  --reload              Restart workers when code changes. [False]
  --reload-engine STRING
                        The implementation that should be used to power :ref:`reload`. [auto]
  --reload-extra-file FILES
                        Extends :ref:`reload` option to also watch and reload on additional files [[]]
  --spew                Install a trace function that spews every line executed by the server. [False]
  --check-config        Check the configuration. [False]
  --preload             Load application code before the worker processes are forked. [False]
  --no-sendfile         Disables the use of ``sendfile()``. [None]
  --reuse-port          Set the ``SO_REUSEPORT`` flag on the listening socket. [False]
  --chdir CHDIR         Chdir to specified directory before apps loading. [/home/user]
  -D, --daemon          Daemonize the Gunicorn process. [False]
  -e ENV, --env ENV     Set environment variable (key=value). [[]]
  -p FILE, --pid FILE   A filename to use for the PID file. [None]
  --worker-tmp-dir DIR  A directory to use for the worker heartbeat temporary file. [None]
  -u USER, --user USER  Switch worker processes to run as this user. [1000]
  -g GROUP, --group GROUP
                        Switch worker process to run as this group. [1000]
  -m INT, --umask INT   A bit mask for the file mode on files written by Gunicorn. [0]
  --initgroups          If true, set the worker process's group access list with all of the [False]
  --forwarded-allow-ips STRING
                        Front-end's IPs from which allowed to handle set secure headers. [127.0.0.1]
  --access-logfile FILE
                        The Access log file to write to. [None]
  --disable-redirect-access-to-syslog
                        Disable redirect access logs to syslog. [False]
  --access-logformat STRING
                        The access log format. [%(h)s %(l)s %(u)s %(t)s "%(r)s" %(s)s %(b)s "%(f)s" "%(a)s"]
  --error-logfile FILE, --log-file FILE
                        The Error log file to write to. [-]
  --log-level LEVEL     The granularity of Error log outputs. [info]
  --capture-output      Redirect stdout/stderr to specified file in :ref:`errorlog`. [False]
  --logger-class STRING
                        The logger you want to use to log events in Gunicorn. [gunicorn.glogging.Logger]
  --log-config FILE     The log config file to use. [None]
  --log-config-dict LOGCONFIG_DICT
                        The log config dictionary to use, using the standard Python [{}]
  --log-syslog-to SYSLOG_ADDR
                        Address to send syslog messages. [udp://localhost:514]
  --log-syslog          Send *Gunicorn* logs to syslog. [False]
  --log-syslog-prefix SYSLOG_PREFIX
                        Makes Gunicorn use the parameter as program-name in the syslog entries. [None]
  --log-syslog-facility SYSLOG_FACILITY
                        Syslog facility name [user]
  -R, --enable-stdio-inheritance
                        Enable stdio inheritance. [False]
  --statsd-host STATSD_ADDR
                        ``host:port`` of the statsd server to log to. [None]
  --dogstatsd-tags DOGSTATSD_TAGS
                        A comma-delimited list of datadog statsd (dogstatsd) tags to append to statsd metrics. []
  --statsd-prefix STATSD_PREFIX
                        Prefix to use when emitting statsd metrics (a trailing ``.`` is added, []
  -n STRING, --name STRING
                        A base to use with setproctitle for process naming. [None]
  --pythonpath STRING   A comma-separated list of directories to add to the Python path. [None]
  --paste STRING, --paster STRING
                        Load a PasteDeploy config file. The argument may contain a ``#`` [None]
  --proxy-protocol      Enable detect PROXY protocol (PROXY mode). [False]
  --proxy-allow-from PROXY_ALLOW_IPS
                        Front-end's IPs from which allowed accept proxy requests (comma separate). [127.0.0.1]
  --keyfile FILE        SSL key file [None]
  --certfile FILE       SSL certificate file [None]
  --ssl-version SSL_VERSION
                        SSL version to use. [_SSLMethod.PROTOCOL_TLS]
  --cert-reqs CERT_REQS
                        Whether client certificate is required (see stdlib ssl module's) [VerifyMode.CERT_NONE]
  --ca-certs FILE       CA certificates file [None]
  --suppress-ragged-eofs
                        Suppress ragged EOFs (see stdlib ssl module's) [True]
  --do-handshake-on-connect
                        Whether to perform SSL handshake on socket connect (see stdlib ssl module's) [False]
  --ciphers CIPHERS     SSL Cipher suite to use, in the format of an OpenSSL cipher list. [None]
  --paste-global CONF   Set a PasteDeploy global config variable in ``key=value`` form. [[]]
  --strip-header-spaces
                        Strip spaces present between the header name and the the ``:``. [False]
╭─<solar_iter_api> [user ~] - [2022-07-09 13:50:56]

```

## config

```text
    -k/--worker-class    worker 类型
    -w/--workers         worker 数量
    -b/--bind            绑定地址
    -t/--timeout         超时 30
    --reload             热重启
    --keep-alive         在 Keep-Alive 连接上等待的秒数 2
    --log-level          INFO DEBUG

    --backlog             积压数 2048
    --worker-connections  单个 worker 最大连接数
    --max-requests        请求达到这个数值后重启 worker
    --max-requests-jitter 请求达到 max-requests 后允许的扩容


file -c/--config

    bind
    workers
    worker_class
    timeout
    keepalive
    loglevel
    reload

    worker_connections
    max_requests
    max_requests_jitter

```
