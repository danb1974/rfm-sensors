#### Hearbeat test

```
        |head |ln|dt|chk  |
- send   de 5b 01 96 40 10

        |head |ln|dt   |chk  |
- expect de 5b 02 96 00 80 2c
```

```
$ echo -e -n "\xde\x5b\x01\x96\x40\x10" | nc 192.168.1.92 23 | hexdump -C -n 7
00000000  de 5b 02 96 00 80 2c                              |.[....,|
00000007
```
