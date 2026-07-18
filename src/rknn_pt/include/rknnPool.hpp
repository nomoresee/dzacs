#ifndef RKNNPOOL_H
#define RKNNPOOL_H

#include "ThreadPool.hpp"
#include <vector>
#include <iostream>
#include <mutex>
#include <deque>
#include <memory>
#include <chrono>

// rknnModel模型类, inputType模型输入类型, outputType模型输出类型
template <typename rknnModel, typename inputType, typename outputType>
class rknnPool
{
private:
    int threadNum;
    std::string modelPath;

    long long id;
    std::mutex idMtx, queueMtx;
    std::unique_ptr<dpool::ThreadPool> pool;
    std::deque<std::future<outputType>> futs;
    std::vector<std::shared_ptr<rknnModel>> models;
    bool ready;

    int buildModels(const std::string &path,
                    std::vector<std::shared_ptr<rknnModel>> &newModels);
    void drainFuturesLocked();

protected:
    int getModelId();

public:
    rknnPool(const std::string modelPath, int threadNum);
    int init();
    int reinit(const std::string& newModelPath);
    // 模型推理/Model inference
    int put(inputType inputData, int cur_frame_id);
    // int put(inputType inputData);

    // 获取推理结果/Get the results of your inference
    int get(outputType &outputData);
    // 非阻塞获取当前所有已完成任务中的最新结果；尚未完成时立即返回。
    int getLatestReady(outputType &outputData);
    // 当前正在执行或等待执行的推理任务数。
    size_t pending();
    ~rknnPool();
};

template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::rknnPool(const std::string modelPath, int threadNum)
{
    this->modelPath = modelPath;
    this->threadNum = threadNum;
    this->id = 0;
    this->ready = false;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::buildModels(
    const std::string &path,
    std::vector<std::shared_ptr<rknnModel>> &newModels)
{
    try
    {
        newModels.reserve(this->threadNum);
        for (int i = 0; i < this->threadNum; i++)
            newModels.push_back(std::make_shared<rknnModel>(path.c_str()));
    }
    catch (const std::bad_alloc &e)
    {
        std::cout << "Out of memory: " << e.what() << std::endl;
        return -1;
    }

    for (int i = 0; i < this->threadNum; i++)
    {
        int ret = newModels[i]->init(newModels[0]->get_pctx(), i != 0);
        if (ret != 0)
            return ret;
    }
    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
void rknnPool<rknnModel, inputType, outputType>::drainFuturesLocked()
{
    while (!futs.empty())
    {
        try
        {
            futs.front().get();
        }
        catch (const std::exception &e)
        {
            std::cerr << "Inference task failed while draining: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Inference task failed while draining" << std::endl;
        }
        futs.pop_front();
    }
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::init()
{
    if (this->threadNum <= 0)
    {
        std::cerr << "threadNum must be greater than zero" << std::endl;
        return -1;
    }

    try
    {
        this->pool = std::make_unique<dpool::ThreadPool>(this->threadNum);
    }
    catch (const std::bad_alloc &e)
    {
        std::cout << "Out of memory: " << e.what() << std::endl;
        return -1;
    }
    std::vector<std::shared_ptr<rknnModel>> newModels;
    int ret = buildModels(this->modelPath, newModels);
    if (ret != 0)
        return ret;

    models.swap(newModels);
    ready = true;
    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::reinit(const std::string& newModelPath)
{
    printf("[rknnPool] Reinitializing model...\n");
    std::lock_guard<std::mutex> queueLock(queueMtx);

    if (ready && newModelPath == this->modelPath)
    {
        printf("[rknnPool] Requested model is already active\n");
        return 0;
    }

    // No model/context can be destroyed while an old inference is running.
    drainFuturesLocked();

    // Build transactionally. If initialization fails, the active model stays
    // intact and inference can continue with it.
    std::vector<std::shared_ptr<rknnModel>> newModels;
    int ret = buildModels(newModelPath, newModels);
    if (ret != 0)
    {
        fprintf(stderr, "[rknnPool] Failed to initialize new model, keeping current model\n");
        return ret;
    }

    models.swap(newModels);
    this->modelPath = newModelPath;
    {
        std::lock_guard<std::mutex> idLock(idMtx);
        this->id = 0;
    }
    ready = true;
    printf("[rknnPool] Reinit complete\n");
    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::getModelId()
{
    std::lock_guard<std::mutex> lock(idMtx);
    int modelId = id % threadNum;
    id++;
    return modelId;
}

// template <typename rknnModel, typename inputType, typename outputType>
// int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData)
// {
//     std::lock_guard<std::mutex> lock(queueMtx);
//     futs.push(pool->submit(&rknnModel::infer, models[this->getModelId()], inputData));
//     return 0;
// }

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData, int cur_frame_id)
// int rknnPool<rknnModel, inputType, outputType>::put(inputType inputData)

{
    std::lock_guard<std::mutex> lock(queueMtx);
    if (!ready || !pool || models.size() != static_cast<size_t>(threadNum))
        return -1;
    futs.push_back(pool->submit(&rknnModel::infer,
                                models[this->getModelId()],
                                inputData, cur_frame_id));
    // futs.push(pool->submit(&rknnModel::infer, models[this->getModelId()], inputData));

    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::get(outputType &outputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);
    if(futs.empty() == true)
        return 1;
    try
    {
        outputData = futs.front().get();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Inference task failed: " << e.what() << std::endl;
        futs.pop_front();
        return -1;
    }
    catch (...)
    {
        std::cerr << "Inference task failed" << std::endl;
        futs.pop_front();
        return -1;
    }
    futs.pop_front();
    return 0;
}

template <typename rknnModel, typename inputType, typename outputType>
int rknnPool<rknnModel, inputType, outputType>::getLatestReady(
    outputType &outputData)
{
    std::lock_guard<std::mutex> lock(queueMtx);
    bool got_result = false;
    bool got_error = false;

    // 检查所有任务而不等待，允许后提交的帧先完成。按提交顺序遍历，因此一轮
    // 内多个结果就绪时，outputData 最终保留最新的那个结果。
    for (auto it = futs.begin(); it != futs.end();)
    {
        if (it->wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready)
        {
            ++it;
            continue;
        }

        try
        {
            outputData = it->get();
            got_result = true;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Inference task failed: " << e.what() << std::endl;
            got_error = true;
        }
        catch (...)
        {
            std::cerr << "Inference task failed" << std::endl;
            got_error = true;
        }
        it = futs.erase(it);
    }

    if (got_result)
        return 0;
    return got_error ? -1 : 1;
}

template <typename rknnModel, typename inputType, typename outputType>
size_t rknnPool<rknnModel, inputType, outputType>::pending()
{
    std::lock_guard<std::mutex> lock(queueMtx);
    return futs.size();
}

template <typename rknnModel, typename inputType, typename outputType>
rknnPool<rknnModel, inputType, outputType>::~rknnPool()
{
    std::lock_guard<std::mutex> lock(queueMtx);
    ready = false;
    drainFuturesLocked();
}

#endif
