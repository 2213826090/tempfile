from .common import GraphicsTestBase


class TestGraphics(GraphicsTestBase):
    '''
    Graphics Test Class
    '''

    def testDecode_BMP_ColorDepth1bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ColorDepth1bit')

    def testDecode_BMP_ColorDepth24bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ColorDepth24bit')

    def testDecode_BMP_ColorDepth32bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ColorDepth32bit')

    def testDecode_BMP_ImageSize1k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ImageSize1k')

    def testDecode_BMP_ImageSize4M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ImageSize4M')

    def testDecode_BMP_ImageSize8M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ImageSize8M')

    def testDecode_BMP_SemiTransparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_SemiTransparent')

    def testDecode_BMP_Transparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_Transparent')

    def testDecode_GIF(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF')

    def testDecode_GIF_EncodedPerGIF87a(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_EncodedPerGIF87a')

    def testDecode_GIF_EncodedPerGIF89a(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_EncodedPerGIF89a')

    def testDecode_GIF_ColorDepth1bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_ColorDepth1bit')

    def testDecode_GIF_ColorDepth8bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_ColorDepth8bit')

    def testDecode_GIF_ImageSize1k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_ImageSize1k')

    def testDecode_BMP_ImageSize2M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ImageSize2M')

    def testDecode_GIF_Transparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_Transparent')

    def testDecode_GIF_SmallSize(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_SmallSize')

    def testDecode_GIF_BigSize(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_BigSize')

    def testDecode_GIF_Medium(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_GIF_Medium')

    def testDecode_JPEG_Progressive(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_Progressive')

    def testDecode_JPEG_ColorDepth24bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorDepth24bit')

    def testDecode_JPEG_ColorDepth8bit_Gray(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorDepth8bit_Gray')

    def testDecode_JPEG_ColorSpaceAdobeRGB(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpaceAdobeRGB')

    def testDecode_JPEG_ColorSpacesRGB(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpacesRGB')

    def testDecode_JPEG_ColorSpacesYUV411(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpacesYUV411')

    def testDecode_JPEG_ColorSpacesYUV420(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpacesYUV420')

    def testDecode_JPEG_ColorSpacesYUV422(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpacesYUV422')

    def testDecode_JPEG_ColorSpacesYUV444(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ColorSpacesYUV444')

    def testDecode_JPEG_HighEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_HighEncodeQuality')

    def testDecode_JPEG_MediumEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_MediumEncodeQuality')

    def testDecode_JPEG_LowEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_LowEncodeQuality')

    def testDecode_JPEG_FileExtension_jpg(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_FileExtension_jpg')

    def testDecode_JPEG_FileExtension_jpeg(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_FileExtension_jpeg')

    def testDecode_JPEG(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG')

    def testDecode_JPEG_ImageSize2M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_JPEG_ImageSize2M')

    def testDecode_BMP_ImageSize16M_ColorDepth24bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_BMP_ImageSize16M_ColorDepth24bit')

    def testDecode_PNG(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG')

    def testDecode_PNG_ColorDepth1bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_ColorDepth1bit')

    def testDecode_PNG_ColorDepth32bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_ColorDepth32bit')

    def testDecode_PNG_ColorDepth48bit(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_ColorDepth48bit')

    def testDecode_PNG_ImageSize1k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_ImageSize1k')

    def testDecode_PNG_ImageSize2M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_ImageSize2M')

    def testDecode_PNG_SemiTransparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_SemiTransparent')

    def testDecode_PNG_Transparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_PNG_Transparent')

    def testDecode_WEBP_SemiTransparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_SemiTransparent')

    def testDecode_WEBP_Transparent(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_Transparent')

    def testDecode_WEBP_HighEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_HighEncodeQuality')

    def testDecode_WEBP_MediumEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_MediumEncodeQuality')

    def testDecode_WEBP_LowEncodeQuality(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_LowEncodeQuality')

    def testDecode_WEBP_ImageSize1k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_ImageSize1k')

    def testDecode_WEBP_ImageSize2M(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WEBP_ImageSize2M')

    def testDecode_WBmP(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WBMP')

    def testDecode_WBmP_ImageSize2k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WBMP_ImageSize2k')

    def testDecode_WBMP_ImageSize300k(self):
        self.graphics.instr_run_class('BitmapFactoryTest#testDecode_WBMP_ImageSize300k')

    def test_ImageView_ThumbnailMode_BMP_LargeSize8M(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_BMP_LargeSize8M')

    def test_ImageView_ThumbnailMode_BMP_SmallSize1k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_BMP_SmallSize1k')

    def test_ImageView_ThumbnailMode_GIF_1k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_GIF_1k')

    def test_ImageView_ThumbnailMode_GIF_LargeSize4M(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_GIF_LargeSize4M')

    def test_ImageView_ThumbnailMode_JPEG_LargeSize8M(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_JPEG_LargeSize8M')

    def test_ImageView_ThumbnailMode_JPEG_MediumSize(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_JPEG_MediumSize')

    def test_ImageView_ThumbnailMode_PNG_LargeSize10M(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_PNG_LargeSize10M')

    def test_ImageView_ThumbnailMode_PNG_SmallSize1k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_PNG_SmallSize1k')

    def test_ImageView_ThumbnailMode_WBMP_LargeSize300k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_WBMP_LargeSize300k')

    def test_ImageView_ThumbnailMode_WEBP_SmallSize1k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_WEBP_SmallSize1k')

    def test_ImageView_ThumbnailMode_WBMP_SmallSize2k(self):
        self.graphics.instr_run_class('MediaStore_Images_ThumbnailsTest#test_ImageView_ThumbnailMode_WBMP_SmallSize2k')
