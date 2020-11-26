#pragma once
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <CxxTools/thread_pool.hpp>

namespace SCFUSION {

    /*
     * Usage:
     *   With Default Function:
     *      use setFunc() to set the default function.
     *      the default function will be running continuously after resume() is called
     *      use pause() to stop the default function.
     *      runOnThisThread() will pause() the default function and insert the given function to the thread and
     *       resume the default thread.
     *   Without Default Function:
     *      use runOnThisThread() to add function to this thread.
     *      use pause() to wait until the given function is finished.
     */
    class ThreadUnit {
    public:
        ThreadUnit(bool useThread=true):terminated_(true), useThread_(useThread), func_(NULL){
            if(useThread_)
                thread_.reset(new tools::TaskThreadPool(1));
        }
        ~ThreadUnit(){
            if(func_) {
                pause();
                delete func_;
            }
        }


        template <typename Task>
        void setFunc(Task task){
            if(func_) delete func_;
            func_ = new std::function< void() > ( static_cast<std::function< void() >>(task)  );
        }


        template <typename Task>
        void runOnThisThread (Task func){
            if(useThread_){
                pause();
                thread_->runTask(func);
                resume();
            } else
                func();
        }

        void pause(){
            if(useThread_){
                if(!terminated_){
                    terminated_ = true;
                    thread_->waitWorkComplete();
                }
            }
        }

        void resume(){
            if(func_)
            if(useThread_){
                if(terminated_){
                    terminated_ = false;
                    thread_->runTaskWithID(std::bind(&ThreadUnit::runKernel, this));
                }
            }
        }

        bool finished(){
            return thread_->finished();
        }

        std::mutex& getMutex() {
            return mutex_;
        }


        tools::TaskThreadPool* getThread(){return thread_.get();}
    private:
        std::unique_ptr<tools::TaskThreadPool> thread_;
        std::atomic<bool> terminated_;
        bool useThread_;
        std::function< void() > *func_;
        std::mutex mutex_;

        void runKernel(){
            std::unique_lock<std::mutex> lock (mutex_);
            if(func_ != NULL)
            while(terminated_ == false && thread_->checkTaskLeft() == 0){
                thread_->runTask(*func_);
            }
            terminated_ = true;
            lock.unlock();
        }
    };
}