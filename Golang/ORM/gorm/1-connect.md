# GORM 连接数据库

## doc

<github.com/go-sql-driver/mysql>

## 驱动参数

```text
[username[:password]@][protocol[(address)]]/dbname[?param1=value1&...&paramN=valueN]
    protocol    tcp
    param true TRUE True 1
```

| param | type | option param | default | desc |
| :-: | :-: | :-: | :-: | :-: |
| allowAllFiles | bool | true/false | false |  |
| allowCleartextPassword | bool | true/false | false |  |
| allowNativePasswords | bool | true/false | true |  |
| charset | string | none | none | utf8mb4 |
| checkConnLiveness | bool | true/false | true |  |
| collation | string |  |  | utf8mb4_general_ci, utf8mb4_0900_ai_ci |
| clientFoundRows | bool | true/false | false |  |
| loc | string | UTC/LOCAL | UTC |  |
| parseTime | bool | true/false | false | 把DATE DATETIME 转换为 time.Time 而不是 []byte / string |
| maxAllowedPacket | decimal |  |  | 4 << 20 // 4 MiB |
| multiStatements | bool | true/false | false | 如果设置为true，就需要放在第一个参数 |
| timeout | duration |  | OS default |  |
| readTimeout | duration |  | 0 | 30s, 0.5m, 1m30s |
| writeTimeout |  | 0 | 30s, 0.5m, 1m30s |
| rejectReadOnly | bool | true/false | false |  |
| serverPubKey | string | none | none |  |
| tls | bool/string |  | true false skip-verify preferred | |

## 连接示例

```go
func main() {

    dsn := fmt.Sprintf(
        "%s:%s@tcp(%s:%d)/%s?charset=utf8mb4&readTimeout=%s&writeTimeout=%s&loc=Local&parseTime=true",
        config.User, config.Passwd, config.Host, config.Port, config.Name, config.ReadTimeout, config.WriteTimeout,
    )
    sqlDB, err = sql.Open("mysql", dsn)
    if err != nil {
        fmt.Println(err)
    }
    sqlDB.SetConnMaxLifetime(time.Second * time.Duration(config.MaxLifetime))
    sqlDB.SetMaxOpenConns(int(config.MaxOpenConn))
    sqlDB.SetMaxIdleConns(int(config.MaxIdleTime))
    sqlDB.SetConnMaxIdleTime(time.Second * time.Duration(config.MaxIdleTime))

    ormDB, err = gorm.Open(mysql.New(mysql.Config{
        Conn: sqlDB,
    }), &gorm.Config{})

}
```
