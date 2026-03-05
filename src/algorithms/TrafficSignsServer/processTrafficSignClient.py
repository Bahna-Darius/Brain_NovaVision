# Copyright (c) 2026, NovaVision
# All rights reserved.

from src.templates.workerprocess import WorkerProcess
from src.algorithms.TrafficSignsServer.threads.threadTrafficSignClient import threadTrafficSignClient


class processTrafficSignClient(WorkerProcess):
    def __init__(self, queueList, logging, ready_event=None, debugging=False):
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        super(processTrafficSignClient, self).__init__(self.queuesList, ready_event)

    def _init_threads(self):
        th = threadTrafficSignClient(self.queuesList, self.logging, self.debugging)
        self.threads.append(th)
