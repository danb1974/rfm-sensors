#### Software
* install python from windows store
* install setuptools `pip install --user setuptools`
* get esptool `git clone https://github.com/themadinventor/esptool.git`
* build esptool `python setup.py install --user`

#### Flash

* for 32Mb/4MB flash (could also use `-fs 4MB`)
* some boards need `-fm dio`
```
python esptool\esptool.py --port COM7 --baud 230400 write_flash -fs detect -ff 80m 0x00000 esp-link\boot_v1.6.bin 0x1000 esp-link\user1.bin 0x3FC000 esp-link\esp_init_data_default.bin 0x3FE000 esp-link\blank.bin
```

#### Misc
* if bootloop, connect a terminal at 76600 (yup) to see the error
* if normal, you get some information first at 76600 (some say at 74880) then at 115200 (mmm)
