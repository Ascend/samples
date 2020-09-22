/**
 * ============================================================================
 *
 * Copyright (C) 2018, Hisilicon Technologies Co., Ltd. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1 Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *   2 Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *   3 Neither the names of the copyright holders nor the names of the
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 * ============================================================================
 */

#include <unistd.h>
#include <thread>
#include <fstream>
#include <algorithm>
#include "Main.h"
#include "hiaiengine/api.h"
#include <libgen.h>
#include <string.h>

static const uint32_t GRAPH_ID = 1875642181;
int flag = 1;
std::mutex mt;
/**
* @ingroup FasterRcnnDataRecvInterface
* @brief RecvData RecvData Recursion，save the file
* @param [in]
*/
HIAI_StatusT CustomDataRecvInterface::RecvData
    (const std::shared_ptr<void>& message)
{
    std::shared_ptr<std::string> data =
        std::static_pointer_cast<std::string>(message);
    mt.lock();
    flag--;
    mt.unlock();
    return HIAI_OK;
}

// Init and create graph
HIAI_StatusT HIAI_InitAndStartGraph()
{
    // Step1: Global System Initialization before using HIAI Engine
    HIAI_StatusT status = HIAI_Init(0);

    // Step2: Create and Start the Graph
    status = hiai::Graph::CreateGraph("./graph.config");
    if (status != HIAI_OK)
    {
        HIAI_ENGINE_LOG(status, "Fail to start graph");
        return status;
    }

    // Step3
    std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(GRAPH_ID);
    if (nullptr == graph)
    {
        HIAI_ENGINE_LOG("Fail to get the graph-%u", GRAPH_ID);
        return status;
    }
	int leaf_array[1] = {601};  //leaf node id

	for(int i = 0;i < 1;i++){
		hiai::EnginePortID target_port_config;
			target_port_config.graph_id = GRAPH_ID;
			target_port_config.engine_id = leaf_array[i];  
			target_port_config.port_id = 0;
			graph->SetDataRecvFunctor(target_port_config,
				std::shared_ptr<CustomDataRecvInterface>(
						 new CustomDataRecvInterface("")));
	}
	return HIAI_OK;
}
int main(int argc, char* argv[])
{
    HIAI_StatusT ret = HIAI_OK;
	char * dirc = strdup(argv[0]);
	if (dirc)
	{
	    char * dname = ::dirname(dirc);
	    chdir(dname);
	    HIAI_ENGINE_LOG("chdir to %s", dname);
	    free(dirc);
	}
    // 1.create graph
    ret = HIAI_InitAndStartGraph();
    if (HIAI_OK != ret)
    {
        HIAI_ENGINE_LOG("Fail to start graph");;
        return -1;
    }

    // 2.send data
    std::shared_ptr<hiai::Graph> graph = hiai::Graph::GetInstance(GRAPH_ID);
    if (nullptr == graph)
    {
        HIAI_ENGINE_LOG("Fail to get the graph-%u", GRAPH_ID);
        return -1;
    }
    
    // send data to SourceEngine 0 port 
    hiai::EnginePortID engine_id;
    engine_id.graph_id = GRAPH_ID;
    engine_id.engine_id = 958; 
    engine_id.port_id = 0;
    std::shared_ptr<std::string> src_data(new std::string);
    graph->SendData(engine_id, "string", std::static_pointer_cast<void>(src_data));
	for (;;)
    {
        if(flag <= 0)
        {
            break;
        }else
        {
            usleep(100000);
        }
    }
    hiai::Graph::DestroyGraph(GRAPH_ID);
    return 0;
}
