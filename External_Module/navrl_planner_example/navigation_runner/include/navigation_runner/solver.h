#ifndef SAFE_ACTION_SOLVER_H
#define SAFE_ACTION_SOLVER_H

#include <navigation_runner/utils.h>
namespace safeAction{
	/**
	 * \brief   Solves a one-dimensional linear program on a specified line subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   planeNo       The plane on which the line lies.
	 * \param   line          The line on which the 1-d linear program is solved
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  True if successful.
	 */
	bool linearProgram1(const std::vector<Plane> &planes, size_t planeNo, const Line &line, double radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		const double dotProduct = line.point * line.direction;
		const double discriminant = sqr(dotProduct) + sqr(radius) - absSq(line.point);

		if (discriminant < 0.0f) {
			/* Max speed sphere fully invalidates line. */
			return false;
		}

		const double sqrtDiscriminant = std::sqrt(discriminant);
		double tLeft = -dotProduct - sqrtDiscriminant;
		double tRight = -dotProduct + sqrtDiscriminant;

		for (size_t i = 0; i < planeNo; ++i) {
			const double numerator = (planes[i].point - line.point) * planes[i].normal;
			const double denominator = line.direction * planes[i].normal;

			if (sqr(denominator) <= EPSILON) {
				/* Lines line is (almost) parallel to plane i. */
				if (numerator > 0.0f) {
					return false;
				}
				else {
					continue;
				}
			}

			const double t = numerator / denominator;

			if (denominator >= 0.0f) {
				/* Plane i bounds line on the left. */
				tLeft = std::max(tLeft, t);
			}
			else {
				/* Plane i bounds line on the right. */
				tRight = std::min(tRight, t);
			}

			if (tLeft > tRight) {
				return false;
			}
		}

		if (directionOpt) {
			/* Optimize direction. */
			if (optVelocity * line.direction > 0.0f) {
				/* Take right extreme. */
				result = line.point + tRight * line.direction;
			}
			else {
				/* Take left extreme. */
				result = line.point + tLeft * line.direction;
			}
		}
		else {
			/* Optimize closest point. */
			const double t = line.direction * (optVelocity - line.point);

			if (t < tLeft) {
				result = line.point + tLeft * line.direction;
			}
			else if (t > tRight) {
				result = line.point + tRight * line.direction;
			}
			else {
				result = line.point + t * line.direction;
			}
		}

		return true;
	}

	/**
	 * \brief   Solves a two-dimensional linear program on a specified plane subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   planeNo       The plane on which the 2-d linear program is solved
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  True if successful.
	 */
	bool linearProgram2(const std::vector<Plane> &planes, size_t planeNo, double radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		const double planeDist = planes[planeNo].point * planes[planeNo].normal;
		const double planeDistSq = sqr(planeDist);
		const double radiusSq = sqr(radius);

		if (planeDistSq > radiusSq) {
			/* Max speed sphere fully invalidates plane planeNo. */
			return false;
		}

		const double planeRadiusSq = radiusSq - planeDistSq;

		const Vector3 planeCenter = planeDist * planes[planeNo].normal;

		if (directionOpt) {
			/* Project direction optVelocity on plane planeNo. */
			const Vector3 planeOptVelocity = optVelocity - (optVelocity * planes[planeNo].normal) * planes[planeNo].normal;
			const double planeOptVelocityLengthSq = absSq(planeOptVelocity);

			if (planeOptVelocityLengthSq <= EPSILON) {
				result = planeCenter;
			}
			else {
				result = planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) * planeOptVelocity;
			}
		}
		else {
			/* Project point optVelocity on plane planeNo. */
			result = optVelocity + ((planes[planeNo].point - optVelocity) * planes[planeNo].normal) * planes[planeNo].normal;

			/* If outside planeCircle, project on planeCircle. */
			if (absSq(result) > radiusSq) {
				const Vector3 planeResult = result - planeCenter;
				const double planeResultLengthSq = absSq(planeResult);
				result = planeCenter + std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
			}
		}

		for (size_t i = 0; i < planeNo; ++i) {
			if (planes[i].normal * (planes[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				/* Compute intersection line of plane i and plane planeNo. */
				Vector3 crossProduct = cross(planes[i].normal, planes[planeNo].normal);

				if (absSq(crossProduct) <= EPSILON) {
					/* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
					return false;
				}

				Line line;
				line.direction = normalize(crossProduct);
				const Vector3 lineNormal = cross(line.direction, planes[planeNo].normal);
				line.point = planes[planeNo].point + (((planes[i].point - planes[planeNo].point) * planes[i].normal) / (lineNormal * planes[i].normal)) * lineNormal;

				if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, result)) {
					return false;
				}
			}
		}

		return true;
	}

	/**
	 * \brief   Solves a three-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes        Planes defining the linear constraints.
	 * \param   radius        The radius of the spherical constraint.
	 * \param   optVelocity   The optimization velocity.
	 * \param   directionOpt  True if the direction should be optimized.
	 * \param   result        A reference to the result of the linear program.
	 * \return  The number of the plane it fails on, and the number of planes if successful.
	 */
	size_t linearProgram3(const std::vector<Plane> &planes, double radius, const Vector3 &optVelocity, bool directionOpt, Vector3 &result)
	{
		if (directionOpt) {
			/* Optimize direction. Note that the optimization velocity is of unit length in this case. */
			result = optVelocity * radius;
		}
		else if (absSq(optVelocity) > sqr(radius)) {
			/* Optimize closest point and outside circle. */
			result = normalize(optVelocity) * radius;
		}
		else {
			/* Optimize closest point and inside circle. */
			result = optVelocity;
		}

		for (size_t i = 0; i < planes.size(); ++i) {
			if (planes[i].normal * (planes[i].point - result) > 0.0f) {
				/* Result does not satisfy constraint i. Compute new optimal result. */
				const Vector3 tempResult = result;

				if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, result)) {
					result = tempResult;
					return i;
				}
			}
		}

		return planes.size();
	}

	/**
	 * \brief   Solves a four-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
	 * \param   planes     Planes defining the linear constraints.
	 * \param   beginPlane The plane on which the 3-d linear program failed.
	 * \param   radius     The radius of the spherical constraint.
	 * \param   result     A reference to the result of the linear program.
	 */
	void linearProgram4(const std::vector<Plane> &planes, size_t beginPlane, double radius, Vector3 &result)
	{
		double distance = 0.0f;

		for (size_t i = beginPlane; i < planes.size(); ++i) {
			if (planes[i].normal * (planes[i].point - result) > distance) {
				/* Result does not satisfy constraint of plane i. */
				std::vector<Plane> projPlanes;

				for (size_t j = 0; j < i; ++j) {
					Plane plane;

					const Vector3 crossProduct = cross(planes[j].normal, planes[i].normal);

					if (absSq(crossProduct) <= EPSILON) {
						/* Plane i and plane j are (almost) parallel. */
						if (planes[i].normal * planes[j].normal > 0.0f) {
							/* Plane i and plane j point in the same direction. */
							continue;
						}
						else {
							/* Plane i and plane j point in opposite direction. */
							plane.point = 0.5f * (planes[i].point + planes[j].point);
						}
					}
					else {
						/* Plane.point is point on line of intersection between plane i and plane j. */
						const Vector3 lineNormal = cross(crossProduct, planes[i].normal);
						plane.point = planes[i].point + (((planes[j].point - planes[i].point) * planes[j].normal) / (lineNormal * planes[j].normal)) * lineNormal;
					}

					plane.normal = normalize(planes[j].normal - planes[i].normal);
					projPlanes.push_back(plane);
				}

				const Vector3 tempResult = result;

				if (linearProgram3(projPlanes, radius, planes[i].normal, true, result) < projPlanes.size()) {
					/* This should in principle not happen.  The result is by definition already in the feasible region of this linear program. If it fails, it is due to small doubleing point error, and the current result is kept. */
					result = tempResult;
				}

				distance = planes[i].normal * (planes[i].point - result);
			}
		}
	}

}
#endif