#!/usr/bin/env python3
"""把 map_server 的 .pgm 地图转成 PNG，并打印尺寸和占用统计，便于不开 RViz 直接看图。

python3 map_to_png.py /go2w/data/maps/lab.yaml [输出.png]
只依赖 numpy 和标准库。
"""
import struct
import sys
import zlib

import numpy as np


def read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    # 头部：P5\n[注释]\nW H\nMAXVAL\n
    tokens, pos = [], 0
    while len(tokens) < 4:
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    w, h = int(tokens[1]), int(tokens[2])
    return np.frombuffer(data[pos + 1:pos + 1 + w * h], dtype=np.uint8).reshape(h, w)


def write_png(path, img):
    h, w = img.shape
    raw = b''.join(b'\x00' + img[i].tobytes() for i in range(h))

    def chunk(tag, body):
        return struct.pack('>I', len(body)) + tag + body + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)

    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n')
        f.write(chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0)))
        f.write(chunk(b'IDAT', zlib.compress(raw, 9)))
        f.write(chunk(b'IEND', b''))


def main():
    yaml_path = sys.argv[1]
    meta = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    pgm = meta['image'] if meta['image'].startswith('/') else \
        yaml_path.rsplit('/', 1)[0] + '/' + meta['image']
    img = read_pgm(pgm)
    res = float(meta['resolution'])
    out = sys.argv[2] if len(sys.argv) > 2 else yaml_path.rsplit('.', 1)[0] + '.png'
    write_png(out, img)
    occ, free = (img < 50).sum(), (img > 250).sum()
    print('%s：%d×%d 像素（%.1f×%.1f m，%.2f m/像素），占用 %d，空闲 %d，未知 %d'
          % (out, img.shape[1], img.shape[0], img.shape[1] * res, img.shape[0] * res, res,
             occ, free, img.size - occ - free))


if __name__ == '__main__':
    main()
