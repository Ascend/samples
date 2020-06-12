/**
*
* Copyright(c)<2018>, <Huawei Technologies Co.,Ltd>
*
* @version 1.0
*
* @date 2018-5-19
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
