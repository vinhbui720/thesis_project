class Pipeline:
    def __init__(self, nodes):
        self.nodes = nodes

    def run(self):
        try:
            while True:
                data = {}
                for node in self.nodes:
                    data = node.process(data)
                    if data is None:
                        return
        finally:
            for node in self.nodes:
                close_fn = getattr(node, "close", None)
                if callable(close_fn):
                    close_fn()