import pytest
from wind_tunnel2 import bus_init, clear_bus, get_bus, read_i2c, write_i2c, PRES_CAL_ADDR, PRES_ADDR

@pytest.fixture
def mock_bus(mocker):
    return mocker.patch('smbus.SMBus')

#=================================================================================================
# TEST bus_init()
#=================================================================================================
class TestInitBus:
    @pytest.fixture
    def reset_bus(self):
        clear_bus()
        
        yield

        clear_bus()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_bus_init_success(self, reset_bus, mock_bus):
        bus_init()
        
        result = get_bus()
        
        assert result is not None

    def test_bus_init_exception(self, reset_bus, mock_bus):
        mock_bus.side_effect = Exception('mocked')
        bus_init()
        
        result = get_bus()
        
        assert result is None

#=================================================================================================
# TEST write_i2c()
#=================================================================================================
class TestWritei2c:
    @pytest.fixture
    def setup_bus(self, mock_bus):
        bus_init()
        
        yield
        
        clear_bus()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_write_i2c_success(self, setup_bus):        
        result = write_i2c(0, 0)

        assert result
    
    def test_write_i2c_exception(self):        
        result = write_i2c("BAD", "INPUT")

        assert not result

#=================================================================================================
# TEST read_i2c()
#=================================================================================================
class TestReadi2c:
    @pytest.fixture
    def setup_bus(self):
        bus_init()
        
        yield
        
        clear_bus()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_i2c_success(self, setup_bus):
        
        result = read_i2c(PRES_ADDR, PRES_CAL_ADDR, 2)

        assert result is not None
    
    def test_read_i2c_bus_exception(self, setup_bus):
        result = read_i2c("VERY", "BAD", "INPUT")

        assert result is None