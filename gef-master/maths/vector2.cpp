#include <maths/vector2.h>
#include <math.h>

namespace gef
{
	const Vector2 Vector2::kZero(0.0f, 0.0f);
	const Vector2 Vector2::kOne(1.0f, 1.0f);

	

	void Vector2::Normalise()
	{
		float length = Length();

		x /= length;
		y /= length;
	}

	float Vector2::LengthSqr() const
	{
		return (x*x + y*y);
	}

	float Vector2::Length() const
	{
		return sqrtf(x*x + y*y);
	}

	Vector2 Vector2::Rotate(float angle)
	{
		Vector2 result;

		result.x = x*cosf(angle) - y*sinf(angle);
		result.y = x*sinf(angle) + y*cosf(angle);

		return result;
	}

	float Vector2::DotProduct(const Vector2& _vec) const
	{
		return x*_vec.x + y*_vec.y;
	}

#pragma region Additional Functionality for Flocking
	void Vector2::Limit(float lim)
	{
		// Reference: https://forum.libcinder.org/topic/limit-the-magnitude-of-vector

		float length = Length();

		if ((length > lim) && (length > 0.0f) && (lim > 0.0f))
		{
			float ratio = lim / length;

			x *= ratio;
			y *= ratio;
		}
	}

	void Vector2::Reset()
	{
		x = 0.0f;
		y = 0.0f;
	}
#pragma endregion
}