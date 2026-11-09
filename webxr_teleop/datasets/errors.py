"""Expected, nonfatal rejection before any accepted-dataset modification."""


class EpisodeRejected(ValueError):
    def __init__(self, reason, path):
        super().__init__(reason)
        self.path = path
