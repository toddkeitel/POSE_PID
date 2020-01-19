#pragma once
//
// NavHelper implements useful Quaternion functions
//
// NavHelper is a stateless templated helper class.
//

#include "Quaternion.h"
#include <math.h>


template<typename T, unsigned int AXIS_COUNT>
class NavHelper
{
public:
	
	struct Position
	{
		// structs are like classes but, they default to public instead of private.

		Position()
		{
			for (int iI = 0; iI < AXIS_COUNT; ++iI)
			{
				m_Position[iI] = T(0);
			}
		}

		Position(T*  _position)
		{
			Set(_position);
		}

		virtual ~Position() {}

		void Set(T* _position)
		{
			for (int iI = 0; iI < AXIS_COUNT; ++iI)
			{
				m_Position[iI] = _position[iI];
			}
		}

		T* Get(void)
		{
			Copy(&m_Position[0], &m_Temp[0]);
			return &m_Temp[0];
		}

		T* Get(unsigned int _index)
		{
			T* retvalue = nullptr;
			if (_index < AXIS_COUNT)
			{
				retvalue = &m_Position[_index];
			}
			return retvalue;
		}

		static void Copy(T* _from, T* _to)
		{
			for (int iI = 0; iI < AXIS_COUNT; ++iI)
			{
				_to[iI] = _from[iI];
			}
		}

	private:
		T m_Position[AXIS_COUNT];
		T m_Temp[AXIS_COUNT];
	};

	NavHelper()
	{
		
	}

	virtual ~NavHelper() {}

	Quaternion<T> Normalize(Quaternion<T>* _quat)
	{
		double sum = double(0);
		for (int iI = 0; iI < Quaternion<T>::eEndOfQuaternionItems; ++iI)
		{
			sum += _quat[iI] * _quat[iI];
		}
		T factor = T(1) / T(sqrt(sum));
		for (int iI = 0; iI < Quaternion<T>::eEndOfQuaternionItems; ++iI)
		{
			m_QuantTempNormalize3.Set(_quat[iI] * factor, iI);
		}
		return m_QuantTempNormalize3;
	}

	Quaternion<T>* Conjucate(Quaternion<T>* _quat)
	{
		m_QuantTempConjucate.Set(_quat->Get()[Quaternion<T>::eW], Quaternion<T>::eW);
		m_QuantTempConjucate.Set(T(-1) * _quat->Get()[Quaternion<T>::eX], Quaternion<T>::eX);
		m_QuantTempConjucate.Set(T(-1) * _quat->Get()[Quaternion<T>::eY], Quaternion<T>::eY);
		m_QuantTempConjucate.Set(T(-1) * _quat->Get()[Quaternion<T>::eZ], Quaternion<T>::eZ);
		return &m_QuantTempConjucate;
	}

	void Transformer(T _transform[][AXIS_COUNT], unsigned int _rank, Position* _input, Position* _output)
	{
		// _transform[col][row] format is assumed

		for (unsigned int iI = 0; iI < _rank; ++iI)
		{
			T* value = _output->Get(iI);
			if (nullptr != value)
			{
				*value = T(0);
			}
		}

		for (unsigned int iI = 0; iI < _rank; ++iI)
		{
			T* outputValue = _output->Get(iI);

			for (unsigned int jJ = 0; jJ < _rank; ++iI)
			{
				T* inputValue = _input->Get(jJ);
				if ((nullptr != outputValue) && (nullptr != inputValue))
				{
					*outputValue += *inputValue * _transform[iI][jJ];
				}
			}
		}
	}
	


private:
	Quaternion<T> m_QuantTempNormalize3;
	Quaternion<T> m_QuantTempConjucate;
};


