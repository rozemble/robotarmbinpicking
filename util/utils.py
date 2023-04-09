import colorsys
import random
import io
from os.path import exists,join
import os
import zlib
import json
import numpy as np
import math
import base64
import cv2

def compress_nparr(nparr):
    """
    Returns the given numpy array as compressed bytestring,
    the uncompressed and the compressed byte size.
    """
    bytestream = io.BytesIO()
    np.save(bytestream, nparr)
    uncompressed = bytestream.getvalue()
    compressed = zlib.compress(uncompressed)
    return compressed, len(uncompressed), len(compressed)

def uncompress_nparr(bytestring):
    """
    """
    return np.load(io.BytesIO(zlib.decompress(bytestring)))

def encodeBase64(data):
    b64_data = base64.b64encode(data).decode("utf-8")
    return b64_data

def shrinkDepth(arr):
    compressed, _, _ = compress_nparr(arr)
    return encodeBase64(compressed)

class MultiEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj,np.int64):
            return int(obj)
        return json.JSONEncoder.default(self, obj)

def random_colors(N, bright=True):
    brightness = 1.0 if bright else 0.7
    hsv = [(i / N, 1, brightness) for i in range(N)]
    colors = list(map(lambda c: colorsys.hsv_to_rgb(*c), hsv))
    random.shuffle(colors)
    return colors

def saveJson(path,fn,data):
    if not exists(path):
        os.makedirs(path)
    with open(join(path,fn+".json"), "w") as json_file:
        json.dump(data, json_file, indent=4, cls=MultiEncoder)