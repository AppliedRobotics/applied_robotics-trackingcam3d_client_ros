#include "tlive_server.h"

namespace tlive {


Server::Server()
{
}

void Server::open()
{
	video_soc_context = zmq_ctx_new();
    video_soc = zmq_socket(video_soc_context, ZMQ_PUB);
	int zmq_hwm = 5;
	zmq_setsockopt(video_soc, ZMQ_SNDHWM, &zmq_hwm, sizeof(zmq_hwm));
    zmq_bind(video_soc, "tcp://*:5563");

	ctrl_soc_context = zmq_ctx_new();
    ctrl_soc = zmq_socket(ctrl_soc_context, ZMQ_REP);
	zmq_bind(ctrl_soc, "tcp://*:5564");
}

int Server::send_data(void *data, size_t size, void (*free_cb)(void* data, void* hint), void *hint)
{
	zmq_msg_t msg;
	zmq_msg_init_data (&msg, data, size, free_cb, hint);
	zmq_sendmsg(video_soc, &msg, ZMQ_DONTWAIT);
	return 0;
}

int Server::ctrl_send(const void *buf, size_t len)
{
	return zmq_send(ctrl_soc, buf, len, ZMQ_DONTWAIT);
}

int Server::ctrl_recv(void *buf, size_t len)
{
	return zmq_recv(ctrl_soc, buf, len, ZMQ_NOBLOCK);
}


};