"""
A module to handle all things Quaternion related.

"""

import numpy as np
import math


class Quaternion:
    def __init__(self, w, x, y, z):
        """
        Args:
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

    __repr__ = __str__

    def __eq__(self, other):
        """
        Test the equality of one quaternion to another. If one of the types
        isn't a quaternion it will return false.

        Args:
            * other {``any``} -- The object to compare against.

        Returns:
            * {``bool``} -- Whether the two quaternions are equal.

        """
        if not isinstance(other, Quaternion):
            return False

        return self.w == other.w and self.x == other.x and self.y == other.y and self.z == other.z

    def __add__(self, other):
        """
        Add two quaternions together.

        Args:
            * other {``Quaternion``} -- The quaternion to add.

        Returns:
            * {``Quaternion``} -- The sum of the two quaternions.

        """
        if not isinstance(other, Quaternion):
            raise TypeError(f"Addition is not defined for types Quaternion and {type(other)}")

        return Quaternion(self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z)

    __radd__ = __add__

    def __sub__(self, other):
        """
        Subtract one quaternion from another.

        Args:
            * other {``Quaternion``} -- The quaternion to subtract.

        Returns:
            * {``Quaternion``} -- The difference of the two quaternions.

        """
        return Quaternion.__add__(self, -other)

    def __rsub__(self, other):
        """This is just to customise our Error message"""
        raise TypeError(f"Subtraction is not defined for types Quaternion and {type(other)}")

    def __iadd__(self, other):
        """
        Add another quaternion to this one in place.

        Args:
            * other {``Quaternion``} -- The quaternion to add.

        Returns:
            * {``Quaternion``} -- The sum of the two quaternions.

        """
        return Quaternion.__add__(self, other)

    def __mul__(self, other):
        """
        Multiply a quaternion by another quaternion or a scalar factor.

        Args:
            * other {``Quaternion`` or ``number``} -- The quaternion or
              scalar to multiply by.

        Returns:
            * {``Quaternion``} -- The product of the multiplication.

        """
        if isinstance(other, (float, int)):
            return Quaternion(self.w * other, self.x * other, self.y * other, self.z * other)

        if isinstance(other, Quaternion):
            return Quaternion(
                self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
                self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
                self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
                self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
            )

        raise TypeError(
            f"Multiplication is not supported between types Quaternion and {type(other)}"
        )

    def __rmul__(self, other):
        """
        Multiply a scalar factor by a quaternion. Quaternion-scalar
        multiplication is commutative, so this reverses the argument order
        and delegates to :meth:`__mul__`.

        Args:
            * other {``number``} -- The scalar to multiply by.

        Returns:
            * {``Quaternion``} -- The product of the multiplication.

        """
        if isinstance(other, (float, int)):
            return Quaternion.__mul__(self, other)

        raise TypeError(
            f"Multiplication is not supported between types Quaternion and {type(other)}"
        )

    def __abs__(self):
        """
        Compute the modulus (magnitude) of the quaternion.

        Returns:
            * {``float``} -- The modulus of the quaternion.

        """
        return np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)

    def conjugate(self):
        """
        Find the conjugate of a quaternion.

        Returns:
            * {``Quaternion``} -- The conjugate of the quaternion.

        """
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def __len__(self):
        """
        All quaternions are of length 4.

        Returns:
            * {``int``} -- Always 4.

        """
        return 4

    def __neg__(self):
        """
        Generate the negative of a quaternion.

        Returns:
            * {``Quaternion``} -- The negated quaternion.

        """
        return Quaternion(-self.w, -self.x, -self.y, -self.z)

    def to_euler_angles(self, *, tol=10e-6):
        """
        Convert a unit quaternion to Euler angles (roll, pitch, yaw), using
        the aerospace z-y'-x'' (yaw-pitch-roll) convention.

        Args:
            * tol {``number``} -- The tolerance allowed between the
              quaternion's modulus and 1 for it to be considered a valid
              unit quaternion. Defaults to ``10e-6``.

        Returns:
            * {``tuple``} -- The ``(phi, m, n)`` Euler angles in radians,
              representing roll, pitch and yaw respectively.

        """
        if abs(abs(self) - 1) > tol:
            raise ValueError(
                f"Unable to convert a quaternion of modulus {abs(self)} to "
                f"euler angles. modulus must be 1 +/- {tol}"
            )

        phi = math.atan2(
            2 * (self.w * self.x + self.y * self.z),
            (1 - 2 * (self.x**2 + self.y**2)),
        )
        m = math.asin(2 * (self.w * self.y - self.z * self.x))

        n = math.atan2(
            2 * (self.w * self.z + self.x * self.y),
            (1 - 2 * (self.y**2 + self.z**2)),
        )

        return (phi, m, n)

    @classmethod
    def from_eul_angles(cls, phi, m, n):
        """
        Generate a rotation quaternion from a set of Euler angles (roll,
        pitch, yaw), using the aerospace z-y'-x'' (yaw-pitch-roll)
        convention. This is the inverse of :meth:`to_euler_angles`.

        Args:
            * phi {``number``} -- The roll angle in radians.
            * m {``number``} -- The pitch angle in radians.
            * n {``number``} -- The yaw angle in radians.

        Returns:
            * {``Quaternion``} -- The rotation quaternion

        """
        norm = np.linalg.norm([phi, m, n])

        if norm == 0:
            return Quaternion(1, 0, 0, 0)

        return cls(
            np.cos(norm),
            phi / norm * np.sin(norm),
            m / norm * np.sin(norm),
            n / norm * np.sin(norm),
        )

    @classmethod
    def from_axis_angle(cls, axis, angle: float) -> "Quaternion":
        """
        Generate a rotation quaternion representing a rotation of ``angle``
        radians about ``axis``.

        Args:
            * axis {``array-like``} -- The 3-vector axis to rotate about.
              Does not need to be normalised; the zero vector returns the
              identity quaternion regardless of ``angle``.
            * angle {``number``} -- The rotation angle, in radians.

        Returns:
            * {``Quaternion``} -- The rotation quaternion.

        """
        axis = np.asarray(axis, dtype=float)
        norm = np.linalg.norm(axis)

        if norm == 0:
            return cls(1, 0, 0, 0)

        axis = axis / norm
        half = angle / 2

        return cls(np.cos(half), *(axis * np.sin(half)))
