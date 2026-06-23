from kb_skills.mutation_client import KnowledgeCoreMutationClient


class _FakeLogger:
    def __init__(self):
        self.messages = []

    def warn(self, message):
        self.messages.append(("warn", message))


class _FakeResponse:
    def __init__(self, success=True, error_msg=""):
        self.success = success
        self.error_msg = error_msg


class _FakeFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response

    def cancel(self):
        return None


class _FakeClient:
    def __init__(self, response=None, ready=True):
        self._response = response or _FakeResponse()
        self._ready = ready
        self.requests = []

    def service_is_ready(self):
        return self._ready

    def call_async(self, request):
        self.requests.append(request)
        return _FakeFuture(self._response)


class _FakeNode:
    def __init__(self, client):
        self._client = client
        self._logger = _FakeLogger()
        self.destroyed_clients = []

    def create_client(self, _srv_type, _service_name, callback_group=None):
        del callback_group
        return self._client

    def get_logger(self):
        return self._logger

    def destroy_client(self, client):
        self.destroyed_clients.append(client)


class _FakeDuration:
    def __init__(self):
        self.sec = 0
        self.nanosec = 0


class _FakeReviseRequest:
    def __init__(self):
        self.method = ""
        self.statements = []
        self.models = []
        self.lifespan = _FakeDuration()


class _FakeRevise:
    class Request(_FakeReviseRequest):
        pass


def test_coerce_statements_accepts_strings_or_lists():
    assert KnowledgeCoreMutationClient.coerce_statements("a rdf:type Book") == [
        "a rdf:type Book"
    ]
    assert KnowledgeCoreMutationClient.coerce_statements(
        ["a rdf:type Book", "", "b rdf:type Human"]
    ) == ["a rdf:type Book", "b rdf:type Human"]


def test_mutation_client_builds_add_request(monkeypatch):
    monkeypatch.setattr("kb_skills.mutation_client.Revise", _FakeRevise)
    client = _FakeClient()
    node = _FakeNode(client)
    mutation_client = KnowledgeCoreMutationClient(node=node, timeout_sec=0.1)

    result = mutation_client.add_fact(
        "book1 rdf:type Book",
        models=["default"],
        lifespan_sec=4.5,
    )

    assert result.success is True
    assert result.dispatched is True
    request = client.requests[0]
    assert request.method == "add"
    assert request.statements == ["book1 rdf:type Book"]
    assert request.models == ["default"]
    assert request.lifespan.sec == 4
    assert request.lifespan.nanosec == 500000000


def test_mutation_client_remove_reports_service_failures(monkeypatch):
    monkeypatch.setattr("kb_skills.mutation_client.Revise", _FakeRevise)
    client = _FakeClient(response=_FakeResponse(success=False, error_msg="boom"))
    node = _FakeNode(client)
    mutation_client = KnowledgeCoreMutationClient(node=node, timeout_sec=0.1)

    result = mutation_client.remove_fact("book1 rdf:type Book")

    assert result.success is False
    assert result.dispatched is True
    assert result.error_msg == "boom"
    assert client.requests[0].method == "retract"


def test_mutation_client_keeps_public_remove_operation_label(monkeypatch):
    monkeypatch.setattr("kb_skills.mutation_client.Revise", _FakeRevise)
    client = _FakeClient()
    node = _FakeNode(client)
    mutation_client = KnowledgeCoreMutationClient(node=node, timeout_sec=0.1)

    result = mutation_client.remove_fact("book1 rdf:type Book")

    assert result.success is True
    assert result.operation == "remove"
    assert client.requests[0].method == "retract"


def test_mutation_client_returns_unavailable_when_service_not_ready(monkeypatch):
    monkeypatch.setattr("kb_skills.mutation_client.Revise", _FakeRevise)
    client = _FakeClient(ready=False)
    node = _FakeNode(client)
    mutation_client = KnowledgeCoreMutationClient(node=node, timeout_sec=0.1)

    result = mutation_client.revise_fact("book1 rdf:type Book")

    assert result.success is False
    assert result.dispatched is False
    assert "unavailable" in result.error_msg.lower()


def test_mutation_client_close_releases_ros_client(monkeypatch):
    monkeypatch.setattr("kb_skills.mutation_client.Revise", _FakeRevise)
    client = _FakeClient()
    node = _FakeNode(client)
    mutation_client = KnowledgeCoreMutationClient(node=node)

    mutation_client.close()

    assert node.destroyed_clients == [client]
