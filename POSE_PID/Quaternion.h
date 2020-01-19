#pragma once
// Basic Quaternion class
//
#include <math.h>

template<typename T>
class Quaternion
{
public:
	enum QuaternionItems
	{
		eX = 0,
		eY,
		eZ,
		eW,
		eEndOfQuaternionItems
	};

	Quaternion()
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Q[iI] = T(0);
		}
	}

	Quaternion(T* _quaternion)
	{
		Set(_quaternion);
	}

	virtual ~Quaternion() {}

	void Set(T* _quaternion)
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Q[iI] = _quaternion[iI];
		}
	}

	void Set(T _angle, T* _axis)
	{
		m_Q[eW] = std::cos(_angle / T(2));
		T sinfactor = std::sin(angle / T(2));
		m_Q[eX] = sinfactor * _axis[eX];
		m_Q[eY] = sinfactor * _axis[eY];
		m_Q[eZ] = sinfactor * _axis[eZ];
	}

	bool Set(T _value, int _index)
	{
		bool success = false;
		if ((_index >= eX) && (_index <= eW))
		{
			Set(_value, static_cast<QuaternionItems(_index));
			success = true;
		}
		return success;
	}

	void Set(T _value, enum QuaternionItems _qtype)
	{
		m_Q[_qtype] = _value;
	}

	Quaternion<T> operator= (Quaternion<T> _from)
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Q[iI] = _from->Get()[iI];
		}
		reurn* this;
	}

	Quaternion<T> operator* ( Quaternion<T> _rhs)
	{
		m_Temp[eW] = _rhs.Get()[eW] - (m_Q[eX] * _rhs.Get()[eX])
									- (m_Q[eY] * _rhs.Get()[eY])
									- (m_Q[eZ] * _rhs.Get()[eZ]);

		m_Temp[eX] = _rhs.Get()[eX] + (m_Q[eX] * _rhs.Get()[eW])
							1		+ (m_Q[eY] * _rhs.Get()[eZ])
									- (m_Q[eZ] * _rhs.Get()[eY]);

		m_Temp[eY] = _rhs.Get()[eY] + (m_Q[eY] * _rhs.Get()[eW])
									+ (m_Q[eZ] * _rhs.Get()[eX])
									- (m_Q[eX] * _rhs.Get()[eZ]);

		m_Temp[eZ] = _rhs.Get()[eZ] + (m_Q[eZ] * _rhs.Get()[eW])
									+ (m_Q[eX] * _rhs.Get()[eY])
									- (m_Q[eY] * _rhs.Get()[eX]);

		return m_Temp;
	}


	T* Get()
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Temp[iI] = m_Q[iI];
		}
		return &m_Temp[0];
	}

private:
	T m_Q[4];
	T m_Temp;
	Quaternion m_Temp;

};