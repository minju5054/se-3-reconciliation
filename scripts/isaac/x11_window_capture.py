"""Read only the Isaac client window; no desktop capture or input injection."""

import ctypes as C
import re
import subprocess

from PIL import Image


class XImage(C.Structure):
    _fields_ = [("width", C.c_int), ("height", C.c_int), ("xoffset", C.c_int),
                ("format", C.c_int), ("data", C.c_void_p), ("byte_order", C.c_int),
                ("bitmap_unit", C.c_int), ("bitmap_bit_order", C.c_int),
                ("bitmap_pad", C.c_int), ("depth", C.c_int), ("bytes_per_line", C.c_int),
                ("bits_per_pixel", C.c_int), ("red_mask", C.c_ulong),
                ("green_mask", C.c_ulong), ("blue_mask", C.c_ulong)]


class IsaacWindowCapture:
    def __init__(self):
        listing = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
        windows = re.findall(r'(0x[0-9a-f]+).*\("IsaacSim"', listing)
        if len(windows) != 1:
            raise RuntimeError(f"expected one Isaac client window, found {windows}")
        self.window = int(windows[0], 16)
        self.x = C.CDLL("libX11.so.6")
        self.composite = C.CDLL("libXcomposite.so.1")
        self.x.XOpenDisplay.argtypes = [C.c_char_p]
        self.x.XOpenDisplay.restype = C.c_void_p
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError("cannot open X display")
        self.composite.XCompositeNameWindowPixmap.argtypes = [C.c_void_p, C.c_ulong]
        self.composite.XCompositeNameWindowPixmap.restype = C.c_ulong
        self.x.XGetGeometry.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong),
            C.POINTER(C.c_int), C.POINTER(C.c_int), *([C.POINTER(C.c_uint)] * 4)]
        self.x.XGetImage.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int,
                                    C.c_uint, C.c_uint, C.c_ulong, C.c_int]
        self.x.XGetImage.restype = C.POINTER(XImage)
        self.x.XDestroyImage.argtypes = [C.POINTER(XImage)]
        self.x.XFreePixmap.argtypes = [C.c_void_p, C.c_ulong]
        self.x.XCloseDisplay.argtypes = [C.c_void_p]

    def capture(self, path):
        if path.exists():
            raise FileExistsError(path)
        pixmap = self.composite.XCompositeNameWindowPixmap(self.display, self.window)
        root, x, y = C.c_ulong(), C.c_int(), C.c_int()
        width, height, border, depth = (C.c_uint() for _ in range(4))
        self.x.XGetGeometry(self.display, pixmap, C.byref(root), C.byref(x), C.byref(y),
                           C.byref(width), C.byref(height), C.byref(border), C.byref(depth))
        im = self.x.XGetImage(self.display, pixmap, 0, 0, width, height, C.c_ulong(-1), 2)
        if not im:
            self.x.XFreePixmap(self.display, pixmap)
            raise RuntimeError("Isaac window image unavailable")
        try:
            v = im.contents
            if v.bits_per_pixel != 32 or v.byte_order != 0 or v.red_mask not in (0, 0xff0000):
                raise RuntimeError("unsupported XImage pixel layout")
            pixels = C.string_at(v.data, v.bytes_per_line * v.height)
            Image.frombytes("RGB", (v.width, v.height), pixels, "raw", "BGRX",
                            v.bytes_per_line, 1).save(path, compress_level=1)
            return [v.width, v.height]
        finally:
            self.x.XDestroyImage(im)
            self.x.XFreePixmap(self.display, pixmap)

    def close(self):
        self.x.XCloseDisplay(self.display)
