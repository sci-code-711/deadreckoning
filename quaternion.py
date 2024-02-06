import numpy as np


class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return f"Quaternion ({self.w}, {self.x}i, {self.y}j, {self.z}k)"

    def __add__(self, other):
        if type(other) != Quaternion:
            raise TypeError(
                f"Addition is not defined for types Quaternion and {type(other)}"
            )

        return Quaternion(
            self.w + other.w, self.x + other.x, self.y + other.y, self.z + other.z
        )

    __radd__ = __add__

    def __sub__(self, other):
        return Quaternion.__add__(self, -other)

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
        if type(other) != Quaternion:
            return False

        return (
            self.w == other.w
            and self.x == other.x
            and self.y == other.y
            and self.z == other.z
        )
    
    def __neq__(self, other):
        return not Quaternion.__eq__(self, other)

    def __mul__(self, other):
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
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def __len__(self):
        return 4
    
    def __neg__(self):
        return Quaternion(- self.w, - self.x, - self.y, - self.z)
