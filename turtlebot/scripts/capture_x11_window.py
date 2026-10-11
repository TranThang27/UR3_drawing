"""Capture an actual X11 window without including other desktop windows."""
import ctypes as C
from PIL import Image


class XImage(C.Structure):
    _fields_ = [(name, typ) for name, typ in [
        ('width', C.c_int), ('height', C.c_int), ('xoffset', C.c_int),
        ('format', C.c_int), ('data', C.c_void_p), ('byte_order', C.c_int),
        ('bitmap_unit', C.c_int), ('bitmap_bit_order', C.c_int),
        ('bitmap_pad', C.c_int), ('depth', C.c_int),
        ('bytes_per_line', C.c_int), ('bits_per_pixel', C.c_int)]]


class WindowCapture:
    def __init__(self, window):
        self.window = window
        self.x = C.CDLL('libX11.so.6')
        self.x.XOpenDisplay.restype = C.c_void_p
        self.display = self.x.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('No X11 display')
        self.x.XGetGeometry.argtypes = [C.c_void_p, C.c_ulong, C.POINTER(C.c_ulong),
            C.POINTER(C.c_int), C.POINTER(C.c_int), C.POINTER(C.c_uint),
            C.POINTER(C.c_uint), C.POINTER(C.c_uint), C.POINTER(C.c_uint)]
        self.x.XGetImage.argtypes = [C.c_void_p, C.c_ulong, C.c_int, C.c_int,
                                     C.c_uint, C.c_uint, C.c_ulong, C.c_int]
        self.x.XGetImage.restype = C.POINTER(XImage)
        self.x.XDestroyImage.argtypes = [C.POINTER(XImage)]
        self.x.XCloseDisplay.argtypes = [C.c_void_p]

    def grab(self):
        root, x, y = C.c_ulong(), C.c_int(), C.c_int()
        width, height, border, depth = [C.c_uint() for _ in range(4)]
        exists=self.x.XGetGeometry(self.display, self.window, C.byref(root), C.byref(x),
                                  C.byref(y), C.byref(width), C.byref(height),
                                  C.byref(border), C.byref(depth))
        if not exists or not width.value or not height.value:
            raise RuntimeError('X11 window is unavailable')
        pointer = self.x.XGetImage(self.display, self.window, 0, 0,
                                   width, height, C.c_ulong(-1).value, 2)
        if not pointer:
            raise RuntimeError('Cannot capture RViz window')
        try:
            value = pointer.contents
            if value.bits_per_pixel != 32 or value.byte_order != 0:
                raise RuntimeError('Unsupported X11 pixel format')
            data = C.string_at(value.data, value.bytes_per_line*value.height)
            return Image.frombytes('RGB', (value.width, value.height), data,
                                    'raw', 'BGRX', value.bytes_per_line, 1)
        finally:
            self.x.XDestroyImage(pointer)

    def close(self):
        self.x.XCloseDisplay(self.display)
