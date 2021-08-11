import pytest
from wind_tunnel2.py import bus_init, clear_bus, get_bus, read_i2c, write_i2c

# Dummy exception for mocking
EXCEPT = Exception('mocked')

@pytest.fixture
def mocked_bus_init(self, mocker):
    return mocker.patch('wind_tunnel2.bus_init')

#=================================================================================================
# TEST bus_init()
#=================================================================================================
class TestInitBus:
    @pytest.fixture
    def new_bus(self, mocker):
        clear_bus()

        mocker.patch('smbus.SMbus', return_value = True)
        bus_init()

        yield get_bus()

        clear_bus()

    def test_bus_init_success(self, new_bus):
        assert new_bus is not None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    @pytest.fixture
    def new_exception_bus(self, mocker):
        clear_bus()

        mocker.patch('smbus.SMbus', side_effect = EXCEPT)
        bus_init()

        yield get_bus()

        clear_bus()

    def test_bus_init_exception(self, new_exception_bus):
        assert new_exception_bus is None

#=================================================================================================
# TEST write_i2c()
#=================================================================================================
class TestWritei2c:
    def test_write_i2c_success(self, mocker):
        mocker.patch('bus.write_byte')

        result = write_i2c(0, 0):

        assert result
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_write_i2c_exception(self, mocker_bus_init):
        mocker.patch('bus.write_byte', side_effect = EXCEPT)

        result = write_i2c(0, 0)

        assert not result

#=================================================================================================
# TEST read_i2c()
#=================================================================================================
class TestReadi2c:
    def test_read_i2c_success(self, mocker):
        mocker.patch('int.from_bytes', return_value = 1)

        result = read_i2c(0, 0, 0)

        assert result is not None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_i2c_bus_exception(self, mocker, mocked_bus_init):
        mocker.patch('bus.read_i2c_block_data', side_effect = EXCEPT)

        result = read_i2c(0, 0, 0)

        assert result is None
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_i2c_int_exception(self, mocker, mocked_bus_init):
        mocker.patch('int.from_bytes', side_effect = EXCEPT)

        result = read_i2c(0, 0, 0)

        assert result is None
