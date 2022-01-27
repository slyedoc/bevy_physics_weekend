
# Troubleshooting

## Ubuntu

If you get the following error:

```shell
error: linking with `cc` failed: exit status: 
.....

note: /usr/bin/ld: cannot find -lxcb-render
```

Then you need to install the `libxcb` packages.

```shell
sudo apt-get install libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev

```
