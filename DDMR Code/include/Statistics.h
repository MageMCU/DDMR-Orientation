//
// Carpenter Software
// File: Class Bitwise.h
// Folder: Algebra
//
// Purpose: Github Depository (MageMCU)
//
// Algebra OOP Library
// The math is underneath the namespace
// nmr for Numerics as in numeric computation.
//
// By Jesse Carpenter (carpentersoftware.com)
//
// Testing Platform:
//  * MCU:Atmega328P
//  * IDE:PlatformIO
//  * Editor: VSCode
//
// MIT LICENSE
//

#ifndef Numerics_Statistics_h
#define Numerics_Statistics_h

#include <Arduino.h>

namespace nmr
{
    template <typename real>
    class Statistics
    {
    private:
        real *m_ptrTuplesX;
        // Experimental
        real *m_ptrTuplesY;
        int m_tuplesSize;

        real m_mean;
        real m_variance;
        real m_standardDeviation;
        real m_median;

        // Private Methods
        void m_Mean();
        void m_Variance();
        void m_StandardDeviation();
        void m_Bubblesort();
        int m_Partitian(int lo, int hi);
        void m_Quicksort();
        void m_Swap(int idx1, int idx2);
        // void m_Copy();
        void m_Median();
        bool m_Odd(int value);
        bool m_Limits(int index);

        // Envelopes
        void m_CalcInitData();
        void m_CalcSortData(int size);

    public:
        // Constructor
        Statistics() = default;
        Statistics(real *tuples, int size);
        ~Statistics() = default;

        // Methods
        int GetTuplesSize();
        real GetElement(int index);

        real Average();
        real StandardDeviation();
        real Median();
        void Queue(real value, int index);
        void QueueReset();

        real GetSorted(int index);
    };

    template <typename real>
    Statistics<real>::Statistics(real *tuples, int size)
    {
        // Variables
        m_ptrTuplesX = tuples;
        m_ptrTuplesY = nullptr;
        m_tuplesSize = size;

        // Envelope All results
        m_CalcInitData();
        m_CalcSortData(size);
    }

    template <typename real>
    int Statistics<real>::GetTuplesSize()
    {
        return m_tuplesSize;
    }

    template <typename real>
    real Statistics<real>::GetElement(int index)
    {
        if (m_Limits(index))
            return m_ptrTuplesX[index];
        return NAN;
    }

    template <typename real>
    real Statistics<real>::Average()
    {
        return m_mean;
    }

    template <typename real>
    real Statistics<real>::StandardDeviation()
    {
        return m_standardDeviation;
    }

    template <typename real>
    real Statistics<real>::Median()
    {
        return m_median;
    }

    template <typename real>
    void Statistics<real>::Queue(real value, int index)
    {
        // experimental
        // purpose... See Statistics_T5_Queue()
        m_ptrTuplesX[index] = value;

        // Envelope
        m_CalcInitData();
        // m_CalcSortData() not used here
    }

    template <typename real>
    void Statistics<real>::QueueReset()
    {
        // Used to increase the standard
        // deviation by inserting an offset
        // at the last index of the set....
        //
        // Needed to unstablize older set for testing
        // a newly queued set..., otherwise the older
        // stable set would give a false test....
        real offset = m_mean + (real)10000;
        m_ptrTuplesX[m_tuplesSize - 1] = offset;

        // Envelope
        m_CalcInitData();
        // m_CalcSortData() not used here
    }

    template <typename real>
    real Statistics<real>::GetSorted(int index)
    {
        if (m_Limits(index))
            return m_ptrTuplesX[index];
        return NAN;
    }

    // PRIVATE METHODS

    template <typename real>
    void Statistics<real>::m_Mean()
    {
        real sum = (real)0;
        for (int i = 0; i < m_tuplesSize; i++)
        {
            sum += m_ptrTuplesX[i];
        }
        m_mean = sum / (real)m_tuplesSize;
    }

    template <typename real>
    void Statistics<real>::m_Variance()
    {
        real temp = (real)0;
        real sum = (real)0;
        for (int i = 0; i < m_tuplesSize; i++)
        {
            temp = m_ptrTuplesX[i] - m_mean;
            sum += temp * temp;
        }
        m_variance = sum / ((real)m_tuplesSize - (real)1);
    }

    template <typename real>
    void Statistics<real>::m_StandardDeviation()
    {
        m_standardDeviation = sqrt((real)m_variance);
    }

    template <typename real>
    void Statistics<real>::m_Bubblesort()
    {
        for (int i = 0; i < (m_tuplesSize - 1); i++)
        {
            for (int j = 0; j < (m_tuplesSize - i - 1); j++)
            {
                if (m_ptrTuplesX[j] > m_ptrTuplesX[j + 1])
                    m_Swap(j, (j + 1));
            }
        }
    }

    template <typename real>
    int Statistics<real>::m_Partitian(int lo, int hi)
    {
        real x = m_ptrTuplesX[hi];
        int i = (lo - 1);
        for (int j = lo; j <= (hi - 1); j++)
        {
            if (m_ptrTuplesX[j] <= x)
            {
                i++;
                m_Swap(i, j);
            }
        }
        m_Swap(i + 1, hi);
        return (i + 1);
    }

    template <typename real>
    void Statistics<real>::m_Quicksort()
    {
        int n = m_tuplesSize;
        int hi = n - 1;
        int lo = 0;
        int p;
        int stack[n];
        int top = -1;

        stack[++top] = lo;
        stack[++top] = hi;
        
        while (top >= 0)
        {
            hi = stack[top--];
            lo = stack[top--];
            
            p = m_Partitian(lo, hi);
            
            if ((p - 1) > lo)
            {
                stack[++top] = lo;
                stack[++top] = p - 1;
            }
            
            if ((p + 1) < hi)
            {
                stack[++top] = p + 1;
                stack[++top] = hi;
            }
        }
    }

    template <typename real>
    void Statistics<real>::m_Swap(int idx1, int idx2)
    {
        real temp = m_ptrTuplesX[idx1];
        m_ptrTuplesX[idx1] = m_ptrTuplesX[idx2];
        m_ptrTuplesX[idx2] = temp;
    }

    template <typename real>
    void Statistics<real>::m_Median()
    {
        real median;
        int index;
        if (m_Odd(m_tuplesSize))
        {
            // ODD: math includes the zero index
            // index = ((m_tuplesSize + 1) / 2) - 1;
            index = ((m_tuplesSize) / 2); // Same as above...
            median = /*m_ptrSorted*/ m_ptrTuplesX[index];
        }
        else
        {
            // EVEN: math includes the zero index
            index = (m_tuplesSize / 2) - 1; // confirmed jc
            median = (/*m_ptrSorted*/ m_ptrTuplesX[index] + /*m_ptrSorted*/ m_ptrTuplesX[index + 1]) * (real)0.5;
        }

        m_median = median;
    }

    template <typename real>
    bool Statistics<real>::m_Odd(int value)
    {
        if ((value % 2) != 0)
            return true;
        return false;
    }

    template <typename real>
    bool Statistics<real>::m_Limits(int index)
    {
        if (index >= 0 && index < m_tuplesSize)
            return true;
        return false;
    }

    template <typename real>
    void Statistics<real>::m_CalcInitData()
    {
        // Using unsorted data
        m_Mean();
        m_Variance();
        m_StandardDeviation();
    }

    template <typename real>
    void Statistics<real>::m_CalcSortData(int size)
    {
        // Check data size
        if (size >= 0 && size < 21)
            m_Bubblesort();
        else
            m_Quicksort();

        // Might try Merge Sort..

        // Requires Sort
        m_Median(); 
    }

}

#endif
