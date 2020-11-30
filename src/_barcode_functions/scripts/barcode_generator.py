#!/usr/bin/env python
import pyqrcode
import os

for s in range(7):
    for i in range(1):
        qr_string = '{' + f"\"S\": {s}, \"I\": {i}" + '}'
        img = pyqrcode.create(qr_string, error='L', version = 2, mode = 'binary')
        file_name = qr_string + '.png'
        save_path = os.path.join('./images', file_name)
        img.png(save_path, scale = 8, module_color=[0, 0, 0, 255], background=[0xff, 0xff, 0xff])

# def generateQR(name, num_qr):
#     qr = qrcode.QRCode(
#         version = None, # Sets size of QR, (Auto Size: Set to None and use "fit")
#         error_correction= qrcode.constants.ERROR_CORRECT_L,
#         box_size = 10, # How many pixels each "box" of the QR code is
#         border = 4 # How thick border of the QR is, minimum of 4
#     )

#     qr.add_data('Sequence No: ')
#     qr.make(fit = True) # True if version = None, auto sz

#     method = 'basic'

#     if (method == 'basic'):
#         factory = qrcode.image.svg.SvgImage
#     elif method == 'fragment':
#         factory = qrcode.image.svg.SvgFragmentImage
#     else:
#         factory = qrcode.image.svg.SvgPathImage

#     img = qr.make_image(image_factory = factory) # make QR code
#     img_io = io.BytesIO()
#     img.save(img_io, 'SVG')
#     img_io.seek(0)

#     generateQR("hello", 1)
