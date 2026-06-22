#ifndef SAFE_ACTION_UTILS_H
#define SAFE_ACTION_UTILS_H

namespace safeAction{
    class Vector3 {
        public:
            double val_[3];
            Vector3(double x = 0, double y = 0, double z = 0)
            {
                val_[0] = x;
                val_[1] = y;
                val_[2] = z;
            }

            double operator[](size_t i) const { return val_[i]; }
            double &operator[](size_t i) { return val_[i]; }

            Vector3 operator-(const Vector3 &vector) const
            {
                return Vector3(val_[0] - vector[0], val_[1] - vector[1], val_[2] - vector[2]);
            }

            Vector3 operator+(const Vector3 &vector) const
            {
                return Vector3(val_[0] + vector[0], val_[1] + vector[1], val_[2] + vector[2]);
            }

            inline double operator*(const Vector3 &vector) const
            {
                return val_[0] * vector[0] + val_[1] * vector[1] + val_[2] * vector[2];
            }

            inline Vector3 operator*(double scalar) const
            {
                return Vector3(val_[0] * scalar, val_[1] * scalar, val_[2] * scalar);
            }

            inline Vector3 operator/(double scalar) const
            {
                const double invScalar = 1.0f / scalar;

                return Vector3(val_[0] * invScalar, val_[1] * invScalar, val_[2] * invScalar);
            }
        };

        inline Vector3 operator*(double scalar, const Vector3 &vector)
        {
            return Vector3(scalar * vector[0], scalar * vector[1], scalar * vector[2]);
        }

        inline Vector3 cross(const Vector3 &vector1, const Vector3 &vector2)
        {
            return Vector3(vector1[1] * vector2[2] - vector1[2] * vector2[1], vector1[2] * vector2[0] - vector1[0] * vector2[2], vector1[0] * vector2[1] - vector1[1] * vector2[0]);
        }

        inline double absSq(const Vector3 &vector)
        {
            return vector * vector;
        }

        static double abs(const Vector3 &vector)
        {
            return std::sqrt(vector * vector);
        }

        static Vector3 normalize(const Vector3 &vector)
        {
            return vector / abs(vector);
        }

        inline double sqr(double scalar)
        {
            return scalar * scalar;
        }

	class Plane {
	public:
		/**
		 * \brief   A point on the plane.
		 */
		Vector3 point;

		/**
		 * \brief   The normal to the plane.
		 */
		Vector3 normal;
		Plane() {}
		Plane(const Vector3 &point, const Vector3 &normal) : point(point), normal(normal) {};
	};    
	
    /**
	 * \brief   A sufficiently small positive number.
	 */
	const double EPSILON = 0.00001f;

	/**
	 * \brief   Defines a directed line.
	 */
	class Line {
	public:
		/**
		 * \brief   The direction of the directed line.
		 */
		Vector3 direction;

		/**
		 * \brief   A point on the directed line.
		 */
		Vector3 point;
	};

}
#endif