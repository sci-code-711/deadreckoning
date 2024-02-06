"""
A module to handle all things Quaternion related.

"""


import numpy as np
import math


class Quaternion:
    def __init__(self, w, x, y, z):
        """
        Arguments:
            * w {``number``} -- The real part of the quaternion
            * x {``number``} -- The x component of the quaternion 
            * y {``number``} -- The y component of the quaternion 
            * z {``number``} -- The z component of the quaternion 

        Returns:
            * {``Quaternion``} -- The constructed quaternion
        
        """
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        """
        Represent the quaternion as a string.

        """
        return f"Quaternion ({self.w}, {self.x}i, {self.y}j, {self.z}k)"

    def __add__(self, other):
        """ Add two quaternions together. """
        if type(other) != Quaternion:
            raise TypeError(
                f"Addition is not defined for types Quaternion and {type(other)}"
            )

        return Quaternion(
            self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z
        )

    __radd__ = __add__

    def __sub__(self, other):
        """ Subtract one quaternion from another. """

        return Quaternion.__add__(self, - other)

    def __rsub__(self, other):
        if other == None:
            return Quaternion(-self.w, -self.x, -self.y, -self.z)

        if type(other) != Quaternion:
            raise TypeError(
                f"Subtraction is not defined for types Quaternion and {type(other)}"
            )

    def __iadd__(self, other):
        return Quaternion.__add__(self, other)

    def __eq__(self, other):
        """ 
        Test the equality of one quaternion to another. If one of the types isn't 
        a quaternion it will return false. 
        
        """
        if type(other) != Quaternion:
            return False

        return (
            self.w == other.w
            and self.x == other.x
            and self.y == other.y
            and self.z == other.z
        )
    
    def __neq__(self, other):
        """ Test the not equal condition. """
        return not Quaternion.__eq__(self, other)

    def __mul__(self, other):
        """
        Multiply a quaternion by another quaternion or a scalar factor.
        
        """
        if type(other) == float or type(other) == int:
            return Quaternion(
                self.w * other, self.x * other, self.y * other, self.z * other
            )
        
        if type(other) == Quaternion:
            return Quaternion(
                self.w * other.w
                - self.x * other.x
                - self.y * other.y
                - self.z * other.z,
                self.w * other.x
                + self.x * other.w
                + self.y * other.z
                - self.z * other.y,
                self.w * other.y
                - self.x * other.z
                + self.y * other.w
                + self.z * other.x,
                self.w * other.z
                + self.x * other.y
                - self.y * other.x
                + self.z * other.w,
            )

        if len(other) == 3:
            return Quaternion.__mul__(self, Quaternion(0, *other))

        raise TypeError(
            f"Multiplication is not supported between types Quaternion and {type(other)}"
        )

    def __rmul__(self, other):
        # This is commutable so we can reverse argument order
        if type(other) == float or type(other) == int:
            return Quaternion.__mul__(self, other)

        if len(other) == 3:
            return Quaternion.__mul__(Quaternion(0, *other), self)

        return Quaternion.__mul__(other, self)

    def __abs__(self):
        return np.sqrt([self.w * 2, self.x * 2, self.y * 2, self.z * 2])

    def conjugate(self):
        """
        Find the conjugate of a quaternion.

        Returns:
            * {``Quaternion``} -- The conjugate of the quaternion.

        """
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def __len__(self):
        """ All quaternions are of length 4. """
        return 4
    
    def __neg__(self):
        """ Generate the negative of a quaternion. """
        return Quaternion(- self.w, - self.x, - self.y, - self.z)
    
    def to_euler_angles(self, *, tol=10e-6):
        """ 
        Convert a quaternion to Euler angles.
        
        """
        if abs(abs(self) - 1) > tol:
            raise ValueError(
                f"Unable to convert a quaternion of modulus {abs(self)} to "
                f"euler angles. modulus must be 1 +/- {tol}"
            )

        l = math.degrees(
            math.atan2(
                2 * (self.w * self.x + self.y * self.z),
                (1 - 2 * (self.x**2 + self.y**2)),
            )
        )
        m = math.degrees(math.asin(2 * (self.w * self.y - self.z * self.x)))

        n = math.degrees(
            math.atan2(
                2 * (self.w * self.z + self.x * self.y),
                (1 - 2 * (self.y**2 + self.z**2)),
            )
        )

        return (l, m, n)
    

    @classmethod
    def from_eul_angles(cls, dl, dm, dn):
        """
        Generate a rotation quaternion from a set of euler angles.

        Returns:
            * {``Quaternion``} -- The rotation quaternion
        
        """
        n_norm = np.linalg.norm([dl, dm, dn])
        if n_norm == 0:
            return Quaternion(1, 0, 0, 0)
        
        qw = np.cos(n_norm)
        qx = (dl / n_norm) * np.sin(n_norm)
        qy = (dm / n_norm) * np.sin(n_norm)
        qz = (dn / n_norm) * np.sin(n_norm)

        return cls.__init__(qw, qx, qy, qz)