import math
from typing import Tuple

# 地球半径, 单位: 米
EarthRadius = 6378137
EarthCircumference = math.pi * 2 * EarthRadius
# 地球周长一半
HalfCircumference = EarthCircumference / 2.0
BaseRadian = math.pi / 180
HalfRadian = math.pi / 360


class Operator:
    def __init__(self, zoom: int, tile_size: int = 256):
        self.zoom = zoom
        self.tile_size = tile_size

        self.last_pic_num, self.pic_num = self.get_pic_num()
        self.pixel_num = self.get_pixel_num()
        self.resolution = self.get_resolution()

    def get_pic_num(self) -> Tuple[int, int]:
        """获取单边图片数"""

        return math.pow(2, self.zoom - 1), math.pow(2, self.zoom)

    def get_pixel_num(self) -> int:
        """获取单边像素数"""

        return self.pic_num * self.tile_size

    def get_resolution(self) -> float:
        """获取分辨率 米/每像素"""

        return EarthCircumference / self.pixel_num

    @staticmethod
    def lon_lat_to_meter(longitude: float, latitude: float) -> Tuple[float, float]:
        """地理坐标转投影坐标

        经纬度转为 xy 米
        :param longitude (float): 地理经度
        :param latitude (float): 地理维度
        :return [float, float]: 投影坐标x, 投影坐标y
        """

        temp = math.log(math.tan((90 + latitude) * HalfRadian)) / BaseRadian / 180

        meter_x = longitude * HalfCircumference / 180
        meter_y = HalfCircumference * temp

        return meter_x, meter_y

    def meter_to_pixel_xy(self, meter_x: float, meter_y: float) -> Tuple[float, float]:
        """投影坐标转像素坐标

        米转像素
        :param meter_x (float): 投影坐标 x
        :param meter_y (float): 投影坐标 y
        :return [float, float]: 像素坐标x, 像素坐标y
        """

        pixel_x = (meter_x + HalfCircumference) / self.resolution
        pixel_y = (meter_y + HalfCircumference) / self.resolution

        return pixel_x, pixel_y

    def lon_lat_to_pixel(self, longitude: float, latitude: float) -> Tuple[int, int]:
        """地理坐标转像素坐标

        经纬度转像素
        :param longitude (float): 地理经度
        :param latitude (float): 地理维度
        :return [int, int]: 像素坐标x, 像素坐标y
        """

        pixel_x = round(((longitude + 180) / 360 * self.pixel_num) % self.tile_size)

        tan_v = math.tan(math.radians(latitude))
        cos_v = math.cos(math.radians(latitude))
        log_v = math.log(tan_v + 1 / cos_v)

        pixel_y = round(((1 - log_v / (2 * math.pi)) * self.pixel_num) % self.tile_size)

        return pixel_x, pixel_y

    def pixel_to_lon_lat(self, tile_x: int, tile_y: int, pixel_x: int, pixel_y: int) -> tuple[float, float]:
        """像素坐标, 瓦片坐标转地理坐标

        :param tile_x (int): 瓦片坐标x
        :param tile_y (int): 瓦片坐标y
        :param pixel_x (int): 像素坐标x
        :param pixel_y (int): 像素坐标y
        :return [float, float]: 地理坐标经度, 地理坐标纬度
        """

        longitude = (tile_x + pixel_x / self.tile_size) / self.pic_num * 360 - 180

        lat = (tile_y + pixel_y / self.tile_size) / self.pic_num
        latitude = math.degrees(math.atan(math.sinh(math.pi - 2 * math.pi * lat)))

        return longitude, latitude

    def pixel_to_tile(self, pixel_x: int, pixel_y: int) -> Tuple[int, int]:
        """像素坐标转瓦片坐标

        :param pixel_x (int): 像素坐标x
        :param pixel_y (int): 像素坐标y
        :return [int, int]: 瓦片坐标x, 瓦片坐标y
        """

        tile_x = math.floor(pixel_x / self.tile_size)
        tile_y = math.floor(pixel_y / self.tile_size)

        return tile_x, tile_y

    def lon_lat_to_tile(self, longitude: float, latitude: float) -> tuple[int, int]:
        """地理坐标转瓦片坐标

        经纬度转瓦片
        :param longitude (float): 地理坐标经度
        :param latitude (float): 地理坐标维度
        :return [int, int]: 瓦片坐标x, 瓦片坐标y
        """

        tile_x = int((longitude + 180) / 360 * self.pic_num)
        tile_y = int((1 - math.asinh(math.tan(math.radians(latitude))) / math.pi) * self.last_pic_num)

        return tile_x, tile_y

    def tile_to_lon_lat(self, tile_x: int, tile_y: int) -> Tuple[float, float]:
        """瓦片坐标转地理坐标

        :param tile_x (int): 瓦片坐标x
        :param tile_y (int): 瓦片坐标y
        :return [float, float]: 地理坐标经度, 地理坐标纬度
        """

        longitude = tile_x / self.pic_num * 360 - 180
        latitude = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * tile_y / self.pic_num))))

        return longitude, latitude
