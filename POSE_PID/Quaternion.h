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
		: m_TempQuat(nullptr)
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Q[iI] = T(0);
		}
	}

	Quaternion(T* _quaternion)
		: m_TempQuat(nullptr)
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
		T sinfactor = std::sin(_angle / T(2));
		m_Q[eX] = sinfactor * _axis[eX];
		m_Q[eY] = sinfactor * _axis[eY];
		m_Q[eZ] = sinfactor * _axis[eZ];
	}

	bool Set(T _value, int _index)
	{
		bool success = false;
		if ((_index >= eX) && (_index <= eW))
		{
			Set(_value, static_cast<QuaternionItems>(_index));
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
			m_Q[iI] = _from.Get()[iI];
		}
		return* this;
	}

	Quaternion<T> operator* ( Quaternion<T> _rhs)
	{
		if (nullptr == m_TempQuat)
		{
			m_TempQuat = new Quaternion();
		}

		m_TempQuat->Set(_rhs.Get()[eW]  - (m_Q[eX] * _rhs.Get()[eX])
										- (m_Q[eY] * _rhs.Get()[eY])
										- (m_Q[eZ] * _rhs.Get()[eZ]), eW);

		m_TempQuat->Set(_rhs.Get()[eX]  + (m_Q[eX] * _rhs.Get()[eW])
										+ (m_Q[eY] * _rhs.Get()[eZ])
										- (m_Q[eZ] * _rhs.Get()[eY]), eX);

		m_TempQuat->Set(_rhs.Get()[eY] + (m_Q[eY] * _rhs.Get()[eW])
										+ (m_Q[eZ] * _rhs.Get()[eX])
										- (m_Q[eX] * _rhs.Get()[eZ]), eY);

		m_TempQuat->Set(_rhs.Get()[eZ] + (m_Q[eZ] * _rhs.Get()[eW])
										+ (m_Q[eX] * _rhs.Get()[eY])
										- (m_Q[eY] * _rhs.Get()[eX]), eZ);

		return *m_TempQuat;
	}


	T* Get()
	{
		for (int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			m_Temp[iI] = m_Q[iI];
		}
		return &m_Temp[0];
	}

	T* Get(unsigned int _index)
	{
		T* retvalue = nullptr;
		if (_index < eEndOfQuaternionItems)
		{
			retvalue = &m_Q[_index];
		}
		return retvalue;
	}

	T GetYawError(Quaternion* _prev)
	{
		T qError[eEndOfQuaternionItems];
		for (unsigned int iI = 0; iI < eEndOfQuaternionItems; ++iI)
		{
			qError[iI] = m_Q[iI] - _prev->Get()[iI];
		}
		T sinY_cosp = T(2) * ((qError[eW] * qError[eZ]) + (qError[eX] * qError[eY]));
		T cosY_cosp = T(1) - T(2) * ((qError[eY] * qError[eY]) + (qError[eZ] * qError[eZ]));
		return static_cast<T>(std::atan2(sinY_cosp, cosY_cosp));
	}
private:
	T m_Q[eEndOfQuaternionItems];
	T m_Temp[eEndOfQuaternionItems];
	Quaternion* m_TempQuat;

};

