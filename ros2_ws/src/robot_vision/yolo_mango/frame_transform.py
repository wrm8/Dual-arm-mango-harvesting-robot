"""Image rotation helpers and pixel mapping for portrait RealSense streams."""


def rotate_image(image, rotate):
    """Rotate a numpy image clockwise/counter-clockwise without resizing."""
    import numpy as np

    if rotate == 'none':
        return image
    if rotate == 'cw':
        return np.ascontiguousarray(np.rot90(image, k=3))
    if rotate == 'ccw':
        return np.ascontiguousarray(np.rot90(image, k=1))
    raise ValueError(f'Unsupported rotation: {rotate}')


def rotated_pixel_to_source(x, y, source_width, source_height, rotate):
    """Map a pixel in a rotated image back to the source image coordinates."""
    if rotate == 'none':
        return float(x), float(y)
    if rotate == 'cw':
        return float(y), float(source_height - 1 - x)
    if rotate == 'ccw':
        return float(source_width - 1 - y), float(x)
    raise ValueError(f'Unsupported rotation: {rotate}')
