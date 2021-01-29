#include "UniformLightGridCommon.h"

void createAndCopyBuffer(RenderContext* pRenderContext, Buffer::SharedPtr& pBuffer, Buffer::SharedPtr& pStagingBuffer, uint elementSize, uint elementCount, void* pCpuData, const std::string& bufferName, const std::string& stagingBufferName)
{
    assert(pCpuData);

    // create if current buffer size < requested size
    if (!pBuffer || pBuffer->getElementCount() < elementCount)
    {
        pBuffer = Buffer::createStructured(elementSize, elementCount, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        pBuffer->setName(bufferName);

        pStagingBuffer = Buffer::createStructured(elementSize, elementCount, Resource::BindFlags::None, Buffer::CpuAccess::Write);
        pStagingBuffer->setName(stagingBufferName);
    }

    // copy grid data to gpu
    size_t bufferSize = elementCount * elementSize;
    void* pGpuData = pStagingBuffer->map(Buffer::MapType::Write);
    memcpy(pGpuData, pCpuData, bufferSize);
    pStagingBuffer->unmap();

    pRenderContext->copyBufferRegion(pBuffer.get(), 0, pStagingBuffer.get(), 0, bufferSize);
}
