#include "UniformLightGridCommon.h"

void createAndCopyBuffer(Buffer::SharedPtr& pBuffer, uint elementSize, uint elementCount, const void* pCpuData, const std::string& bufferName)
{
    assert(pCpuData);

    // create if current buffer size < requested size
    if (!pBuffer || pBuffer->getElementCount() < elementCount)
    {
        pBuffer = Buffer::createStructured(elementSize, elementCount, Resource::BindFlags::ShaderResource | Resource::BindFlags::UnorderedAccess, Buffer::CpuAccess::None, nullptr, false);
        pBuffer->setName(bufferName);
    }

    // copy grid data to gpu
    size_t bufferSize = elementCount * elementSize;
    pBuffer->setBlob(pCpuData, 0, bufferSize);
}
