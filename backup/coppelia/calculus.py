import math


class Calculus:
    pi = math.pi

    def __init__(self):
        """
        Class for all necessary calculations
        """
        pass

    @classmethod
    def degree_to_rad(cls, theta):
        return theta * cls.pi/180 if theta else 0

    @classmethod
    def rad_to_degree(cls, theta):
        return round(theta * 180/cls.pi) if theta else 0

    @classmethod
    def format_vertical(cls, target):
        return target/1000 if target else 0

    @classmethod
    def round_cable_position(cls, pos):
        return round(pos * 1000, 1) if pos else 0

    @classmethod
    def round_sensor_distance(cls, distance):
        return round(distance * 1000, 1) if distance else 0
