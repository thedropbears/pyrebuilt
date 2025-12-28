from utilities import caching


def test_cache_per_loop():
    class Test(caching.HasPerLoopCache):
        def __init__(self):
            super().__init__()
            self._bare_called = 0
            self._property_called = 0

        @caching.cache_per_loop
        def get_bare(self):
            self._bare_called += 1
            return self._bare_called

        @property
        @caching.cache_per_loop
        def a_property(self):
            self._property_called += 1
            return self._property_called

    test = Test()
    assert test.get_bare() == 1
    assert test.get_bare() == 1
    assert test.a_property == 1
    assert test.a_property == 1

    assert len(test._per_loop_cache) == 2
    test._per_loop_cache.clear()

    assert test.get_bare() == 2
    assert test.get_bare() == 2
    assert test.a_property == 2
    assert test.a_property == 2
