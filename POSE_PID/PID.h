#pragma once

#include <math.h>
#include <string>

template<typename T, unsigned int AXISCOUNT>
class PID
{
private:
	T m_Kp[AXISCOUNT];
	T m_Ki[AXISCOUNT];
	T m_Kd[AXISCOUNT];
	T m_PrevError[AXISCOUNT];
	T m_DeltaTime;
	T m_Error[AXISCOUNT];
	T m_Output[AXISCOUNT];
	T m_IntegSum[AXISCOUNT];
	unsigned int m_ShowCounter;
	std::string m_OutputString[AXISCOUNT];

	bool ValidIndex(unsigned int _index)
	{
		return _index < AXISCOUNT;
	}

	void ComputeError(T* _cmd, T* _signal, unsigned int _index)
	{
		if (ValidIndex(_index))
		{
			m_Error[_index] = _cmd[_index] - _signal[_index];
		}
	}

	T ExecuteProp(unsigned int _index)
	{
		T term = T(0);
		if (ValidIndex(_index))
		{
			term = m_Kp[_index] * m_Error[_index];
		}
		Show("Proportional Term", term);
		return term;
	}

	T ExecuteInteg(unsigned int _index)
	{
		T term = T(0);
		if (ValidIndex(_index))
		{
			m_IntegSum[_index] += ( m_Error[_index] + m_PrevError[_index] ) * m_DeltaTime / T(2);
			term = m_IntegSum[_index] * m_Ki[_index];
		}
		Show("Integral Term", term);
		return term;
	}

	T ExecuteDeriv(unsigned int _index)
	{
		T term = T(0);
		if (ValidIndex(_index) && T(0) != m_DeltaTime)
		{
			term = (m_Error[_index] - m_PrevError[_index]) / m_DeltaTime;
		}
		Show("Derivative Term", term);
		return term;
	}

	void UpdatePrevValue(unsigned int _index)
	{
		if (ValidIndex(_index))
		{
			m_PrevError[_index] = m_Error[_index];
		}
	}


	void Show(const char* _name, T _value)
	{
		static const std::string delimiter(" , ");

		std::cout << m_ShowCounter << delimiter.c_str() << _name << delimiter.c_str() << _value << std::endl;
	}

public:
	PID(T* _kp, T* _ki, T* _kd, T _deltatime)
		: m_DeltaTime(_deltatime)
		, m_ShowCounter(0)
	{
		m_OutputString[0].assign("Output Vector X-Term");
		m_OutputString[1].assign("Output Vector Y-Term");
		m_OutputString[2].assign("Output Vector Z-Term");
		//This part of the code (m_OutputString) was not compliant with AXISCOUNT, this assignment of outputstring values should be refactored to be compliant
		SetGains(_kp, _ki, _kd);
		Reset();
	}

	virtual ~PID() {}

	void SetGains(T* _kp, T* _ki, T* _kd)
	{
		for (unsigned int iI = 0; iI < AXISCOUNT; ++iI)
		{
			m_Kp[iI] = _kp[iI];
			m_Ki[iI] = _ki[iI];
			m_Kd[iI] = _kd[iI];
		}
	}

	void Reset(void)
	{
		for(unsigned int iI = 0; iI < AXISCOUNT; ++iI)
		{
			m_IntegSum[iI] = T(0);
			m_Error[iI] = T(0);
			m_PrevError[iI] = T(0);
		}
		m_ShowCounter = 0;
	}

	T* Run(T* _signal, T* _cmd)
	{
		for (unsigned int iI = 0; iI < AXISCOUNT; ++iI)
		{
			ComputeError(_cmd, _signal, iI);
			m_Output[iI] = ExecuteProp(iI);
			m_Output[iI] += ExecuteInteg(iI);
			m_Output[iI] += ExecuteDeriv(iI);
			Show(m_OutputString[iI].c_str(), m_Output[iI]);
			//Implement limiters
			//Execute integrator anti-windup, with anti-jitter, using intermediate values (replace m_Output[iI] with unique variables)
			UpdatePrevValue(iI);
			++m_ShowCounter;
		}
		return &m_Output[0];
	}

};

