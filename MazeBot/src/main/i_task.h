#pragma once

#include "project_config.h"
#include <stdint.h>

namespace tasks
{

    // базовый интерфейс таска, от него наследуются все остальные
    // каждый таск должен реализовывать виртуальный метод worker
    // который будет вызываться через предоставляемый интерфейс "work"

    // TTimeType - тип, которым оперирует служба времени
    template < typename TTimeType >
    class ITask
    {
    public:

        typedef TTimeType TimeType;

        // тип вызова
        STRONG_ENUM( CallType, PERIODIC_DELAY, SINGLE_DELAY, NO_DELAY );

        // состояния, по дефолту - Pending
        STRONG_ENUM( TaskState, INACTIVE, PENDING );

        // по-умолчанию создается таск без задержки - т.е. работающий при каждом вызове
        ITask(void) : m_taskState( TaskState::PENDING ),
                      m_taskPeriodicDelayTime(0),
                      m_taskSingleDelayTime(0),
                      m_taskLastCallTime(0),
                      m_taskCurCallTime(0),
                      m_taskCurCallType( CallType::NO_DELAY)
        {;}
        
        virtual ~ITask() {}

        // состояние

        TaskState getState(void)
        {
            return m_taskState;
        }

        bool isPending(void)
        {
            return m_taskState == TaskState::PENDING;
        }

        void makePending(void)
        {
            this->m_taskState = TaskState::PENDING;
        }

        void makeInactive(void)
        {
            this->m_taskState = TaskState::INACTIVE;
        }

        // период вызова
        // не гарантирует время ближайшего вызова!
        void setPeriodicDelay(TimeType delay)
        {
            this->m_taskPeriodicDelayTime = delay;
            this->m_taskCurCallType = CallType::PERIODIC_DELAY;
        }

        void setSingleDelay(TimeType delay, TimeType curTime)
        {
            this->m_taskSingleDelayTime = delay;
            this->m_taskCurCallTime = curTime;
            this->m_taskLastCallTime = curTime;
            this->m_taskCurCallType = CallType::SINGLE_DELAY;
        }

        void clearDelay(void)
        {
            this->m_taskCurCallType = CallType::NO_DELAY;
        }
        

        // основной метод таска, реализуемый в потомках
        void work(TimeType curTime)
        {
            // неактивны - работать не надо
            if( ! this->isPending() )
            {
                return;
            }

            switch(m_taskCurCallType)
            {

            case CallType::NO_DELAY:

                // текущее время может понадобиться в воркере
                m_taskCurCallTime = curTime;

                this->worker();

                m_taskLastCallTime = curTime;

                break;

            case CallType::SINGLE_DELAY:

                if( curTime - m_taskLastCallTime >= m_taskSingleDelayTime )
                {
                    // текущее время может понадобиться в воркере
                    m_taskCurCallTime = curTime;

                    this->worker();

                    m_taskLastCallTime = curTime;

                    // дальше опять периодически
                    m_taskCurCallType = CallType::PERIODIC_DELAY;
                }

                break;

            case CallType::PERIODIC_DELAY:

                // можно считать время ТОЛЬКО так - из-за возможного переполнения счетчика времени
                if( curTime - m_taskLastCallTime >= m_taskPeriodicDelayTime )
                {
                    // текущее время может понадобиться в воркере
                    m_taskCurCallTime = curTime;

                    this->worker();
                    
                    m_taskLastCallTime = curTime;
                }

                break;

            }//switch
        }

    protected:

        // текущее состояние - неактивный или просящийся
        TaskState m_taskState;

        // период вызова автомата по таймеру
        TimeType m_taskPeriodicDelayTime;

        TimeType m_taskSingleDelayTime;

        // время последнего вызова
        TimeType m_taskLastCallTime;

        // время текущего вызова
        TimeType m_taskCurCallTime;

        // тип вызова - с однократной задержкой, периодически или без таймера
        CallType m_taskCurCallType;

    private:

        // этот метод реализуется в потомках и вызывается через публичный "интерфейс" work
        virtual void worker(void) = 0;

    }; // class ITask


}// namespace

