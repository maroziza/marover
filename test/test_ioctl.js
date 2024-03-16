#!/usr/local/bin/qjs
import {ioctl} from "../src/ioctl.so";
import * as os from "os";

const FIONREAD = 0x541B;
var fd = os.open("data/file.txt", os.O_READ)
var nr = ioctl(fd,FIONREAD);

console.log(nr == 5);
