# SQL题目

[TOC]

## 1 查询部门中薪资最高的信息

### 1.0 table

```sql
CREATE TABLE `employee` (
    `id` bigint NOT NULL AUTO_INCREMENT PRIMARY KEY,
    `name` varchar(100),
    `salary` int,
    `department_id` bigint
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4;


CREATE TABLE `departments` (
    `id` bigint NOT NULL AUTO_INCREMENT PRIMARY KEY,
    `name` varchar(100)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4
```

### 1.1 results

```sql
SELECT employee.name, departments.name, employee.salary from employee
INNER JOIN departments ON employee.department_id = departments.id
WHERE (employee.department_id, employee.salary) IN
(
    SELECT employee.department_id, MAX(employee.salary) FROM employee
    GROUP BY employee.department_id
)
```

## 2 找到身份证号重复的数据

```sql
SELECT persons.name, COUNT(persons.sn) FROM persons
GROUP BY persons.sn
HAVING COUNT(persons.sn) > 1
```

## 3 第二高薪水

使用 limit offset，但是没有第二高时，不符合需求

```sql
SELECT DISTINCT Salary AS SecondHighestSalary
FROM Employee
ORDER BY Salary DESC
LIMIT 1 OFFSET 1
```

### 3.1 使用临时表

```sql
SELECT
(
    SELECT DISTINCT Salary
    FROM Employee
    ORDER BY Salary DESC
    LIMIT 1 OFFSET 1
)
AS SecondHighestSalary;
```

### 3.2 使用 IFNULL

```sql
SELECT IFNULL
(
    (
        SELECT DISTINCT Salary
        FROM Employee
        ORDER BY Salary DESC
        LIMIT 1 OFFSET 1
    ),
NULL
) AS SecondHighestSalary;
```

### 3.3 使用子查询

```sql
SELECT MAX(salary) AS SecondHighestSalary
FROM Employee
WHERE salary < (
    SELECT MAX(salary)
    FROM Employee
    WHERE salary
)
```

## 4 查询第N高的薪资

```sql
CREATE FUNCTION getNthHighestSalary(N INT) RETURNS INT
BEGIN
    set N := N-1;
  RETURN (
      # Write your MySQL query statement below.
      select ifnull
      (
          (
            SELECT DISTINCT salary
            from Employee
            order by salary desc
            limit N, 1
          ),
      null
      ) AS getNthHighestSalary
  );
END
```

## 5 没有购过物的人员

### 5.1 使用 not in

```sql
select Customers.Name as Customers
from Customers
where Customers.Id not in (
    select CustomerId from Orders
    group by CustomerId
)
```

### 5.2 使用 not exists

```sql
select name as Customers
  from Customers c
 where not exists (select 1 from Orders o where c.id = o.CustomerId)
```

### 5.3 使用左连接

```sql
select c.Name as Customers
from Customers as c
left join Orders as o on c.Id = o.CustomerId
where o.Id is null
```

## 6 超过经理收入的员工 181

### 6.1 table

```sql
CREATE TABLE IF NOT EXISTS Employee (
    id INT,
    name VARCHAR(255),
    salary INT,
    managerId INT
);

TRUNCATE TABLE Employee;

INSERT INTO Employee (id, name, salary, managerId) VALUES
('1', 'Joe', '70000', '3'),
('2', 'Henry', '80000', '4'),
('3', 'Sam', '60000', 'None'),
('4', 'Max', '90000', 'None');
```

### 6.2 子查询方式

```sql
SELECT a.name AS Employee
FROM Employee a, (SELECT id, salary FROM Employee) b
WHERE a.managerId = b.id AND
a.salary > b.salary;
```

### 6.3 连接查询方式

```sql
SELECT a.name AS Employee
FROM Employee a
LEFT JOIN Employee b ON a.managerId = b.id
WHERE a.salary > b.salary;
```

## 7 查找重复的电子邮箱 182

<https://leetcode.cn/problems/duplicate-emails/>

### 7.1 table

```sql
CREATE TABLE IF NOT EXISTS Person (id int, email varchar(255));

Truncate table Person;

INSERT INTO Person (id, email) values ('1', 'a@b.com')
INSERT INTO Person (id, email) values ('2', 'c@d.com')
INSERT INTO Person (id, email) values ('3', 'a@b.com')
```

### 7.2 SQL

```sql
SELECT email AS Email FROM Person
GROUP BY email
HAVING COUNT(id) > 1;
```

## 8 上升的温度 197

<https://leetcode.cn/problems/rising-temperature/>

### 8.1 table

```sql
CREATE TABLE IF NOT EXISTS Weather (id int, recordDate date, temperature int)
Truncate table Weather
INSERT INTO Weather (id, recordDate, temperature) values ('1', '2015-01-01', '10')
INSERT INTO Weather (id, recordDate, temperature) values ('2', '2015-01-02', '25')
INSERT INTO Weather (id, recordDate, temperature) values ('3', '2015-01-03', '20')
INSERT INTO Weather (id, recordDate, temperature) values ('4', '2015-01-04', '30')
```

### 8.2 SQL

```sql
SELECT a.id FROM
Weather a
LEFT JOIN Weather b ON a.recordDate = ADDDATE(b.recordDate, 1)
WHERE a.temperature > b.temperature
```
