# Hypercorn

## CMD

```text

hypercorn --help

usage: hypercorn [-h] [--access-log ACCESS_LOG] [--access-logfile ACCESS_LOGFILE] [--access-logformat ACCESS_LOGFORMAT]
                 [--backlog BACKLOG] [-b BINDS] [--ca-certs CA_CERTS] [--certfile CERTFILE] [--cert-reqs CERT_REQS]
                 [--ciphers CIPHERS] [-c CONFIG] [--debug] [--error-log ERROR_LOG] [--error-logfile ERROR_LOGFILE]
                 [--graceful-timeout GRACEFUL_TIMEOUT] [--read-timeout READ_TIMEOUT] [-g GROUP] [-k WORKER_CLASS]
                 [--keep-alive KEEP_ALIVE] [--keyfile KEYFILE] [--keyfile-password KEYFILE_PASSWORD]
                 [--insecure-bind INSECURE_BINDS] [--log-config LOG_CONFIG] [--log-level LOG_LEVEL] [-p PID]
                 [--quic-bind QUIC_BINDS] [--reload] [--root-path ROOT_PATH] [--server-name SERVER_NAMES]
                 [--statsd-host STATSD_HOST] [--statsd-prefix STATSD_PREFIX] [-m UMASK] [-u USER]
                 [--verify-mode VERIFY_MODE] [--websocket-ping-interval WEBSOCKET_PING_INTERVAL] [-w WORKERS]
                 application

positional arguments:
  application           The application to dispatch to as path.to.module:instance.path

optional arguments:
  -h, --help            show this help message and exit
  --access-log ACCESS_LOG
                        Deprecated, see access-logfile
  --access-logfile ACCESS_LOGFILE
                        The target location for the access log, use `-` for stdout
  --access-logformat ACCESS_LOGFORMAT
                        The log format for the access log, see help docs
  --backlog BACKLOG     The maximum number of pending connections
  -b BINDS, --bind BINDS
                        The TCP host/address to bind to. Should be either host:port, host, unix:path or fd://num, e.g.
                        127.0.0.1:5000, 127.0.0.1, unix:/tmp/socket or fd://33 respectively.
  --ca-certs CA_CERTS   Path to the SSL CA certificate file
  --certfile CERTFILE   Path to the SSL certificate file
  --cert-reqs CERT_REQS
                        See verify mode argument
  --ciphers CIPHERS     Ciphers to use for the SSL setup
  -c CONFIG, --config CONFIG
                        Location of a TOML config file, or when prefixed with `file:` a Python file, or when prefixed with
                        `python:` a Python module.
  --debug               Enable debug mode, i.e. extra logging and checks
  --error-log ERROR_LOG
                        Deprecated, see error-logfile
  --error-logfile ERROR_LOGFILE, --log-file ERROR_LOGFILE
                        The target location for the error log, use `-` for stderr
  --graceful-timeout GRACEFUL_TIMEOUT
                        Time to wait after SIGTERM or Ctrl-C for any remaining requests (tasks) to complete.
  --read-timeout READ_TIMEOUT
                        Seconds to wait before timing out reads on TCP sockets
  -g GROUP, --group GROUP
                        Group to own any unix sockets.
  -k WORKER_CLASS, --worker-class WORKER_CLASS
                        The type of worker to use. Options include asyncio, uvloop (pip install hypercorn[uvloop]), and
                        trio (pip install hypercorn[trio]).
  --keep-alive KEEP_ALIVE
                        Seconds to keep inactive connections alive for
  --keyfile KEYFILE     Path to the SSL key file
  --keyfile-password KEYFILE_PASSWORD
                        Password to decrypt the SSL key file
  --insecure-bind INSECURE_BINDS
                        The TCP host/address to bind to. SSL options will not apply to these binds. See *bind* for
                        formatting options. Care must be taken! See HTTP -> HTTPS redirection docs.
  --log-config LOG_CONFIG
                        A Python logging configuration file.
  --log-level LOG_LEVEL
                        The (error) log level, defaults to info
  -p PID, --pid PID     Location to write the PID (Program ID) to.
  --quic-bind QUIC_BINDS
                        The UDP/QUIC host/address to bind to. See *bind* for formatting options.
  --reload              Enable automatic reloads on code changes
  --root-path ROOT_PATH
                        The setting for the ASGI root_path variable
  --server-name SERVER_NAMES
                        The hostnames that can be served, requests to different hosts will be responded to with 404s.
  --statsd-host STATSD_HOST
                        The host:port of the statsd server
  --statsd-prefix STATSD_PREFIX
                        Prefix for all statsd messages
  -m UMASK, --umask UMASK
                        The permissions bit mask to use on any unix sockets.
  -u USER, --user USER  User to own any unix sockets.
  --verify-mode VERIFY_MODE
                        SSL verify mode for peer's certificate, see ssl.VerifyMode enum for possible values.
  --websocket-ping-interval WEBSOCKET_PING_INTERVAL
                        If set this is the time in seconds between pings sent to the client. This can be used to keep the
                        websocket connection alive.
  -w WORKERS, --workers WORKERS
                        The number of workers to spawn and use

```
