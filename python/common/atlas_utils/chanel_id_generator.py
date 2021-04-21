import threading


class _ChannelIdGenerator(object):
    """Generate global unique id number, single instance mode class"""
    _instance_lock = threading.Lock()
    channel_id = 0

    def __init__(self):
        pass

    def __new__(cls, *args, **kwargs):
        if not hasattr(_ChannelIdGenerator, "_instance"):
            with _ChannelIdGenerator._instance_lock:
                if not hasattr(_ChannelIdGenerator, "_instance"):
                    _ChannelIdGenerator._instance = object.__new__(
                        cls, *args, **kwargs)
        return _ChannelIdGenerator._instance

    def generator_channel_id(self):
        """Generate global unique id number
        The id number is increase
        """
        curren_channel_id = 0
        with _ChannelIdGenerator._instance_lock:
            curren_channel_id = _ChannelIdGenerator.channel_id
            _ChannelIdGenerator.channel_id += 1

        return curren_channel_id


def gen_unique_channel_id():
    """Interface of generate global unique id number"""
    generator = _ChannelIdGenerator()
    return generator.generator_channel_id()
