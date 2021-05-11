library ieee;
use ieee.std_logic_1164.all;


entity io_expanson is port(
	clk  : in std_logic;
	rstn : in std_logic;
	op_led : out std_logic_vector(7 downto 0);
	i2c_scl : inout std_logic;
	i2c_sda : inout std_logic;
	mosi	: in std_logic;
	ss    : in std_logic;
	miso  : inout std_logic;
	sclk  : in std_logic
	
	);
end entity;

architecture rtl of io_expanson is 


component i2c_base is
	port (
		clk_clk                                                                                         : in    std_logic                    := '0'; --                                             clk.clk
		i2cslave_to_avlmm_bridge_0_conduit_end_conduit_data_in                                          : in    std_logic                    := '0'; --          i2cslave_to_avlmm_bridge_0_conduit_end.conduit_data_in
		i2cslave_to_avlmm_bridge_0_conduit_end_conduit_clk_in                                           : in    std_logic                    := '0'; --                                                .conduit_clk_in
		i2cslave_to_avlmm_bridge_0_conduit_end_conduit_data_oe                                          : out   std_logic;                           --                                                .conduit_data_oe
		i2cslave_to_avlmm_bridge_0_conduit_end_conduit_clk_oe                                           : out   std_logic;                           --                                                .conduit_clk_oe
		pio_0_external_connection_export                                                                : out   std_logic_vector(7 downto 0);        --                       pio_0_external_connection.export
		reset_reset_n                                                                                   : in    std_logic                    := '0'; --                                           reset.reset_n
		spi_slave_to_avalon_mm_master_bridge_0_export_0_mosi_to_the_spislave_inst_for_spichain          : in    std_logic                    := '0'; -- spi_slave_to_avalon_mm_master_bridge_0_export_0.mosi_to_the_spislave_inst_for_spichain
		spi_slave_to_avalon_mm_master_bridge_0_export_0_nss_to_the_spislave_inst_for_spichain           : in    std_logic                    := '0'; --                                                .nss_to_the_spislave_inst_for_spichain
		spi_slave_to_avalon_mm_master_bridge_0_export_0_miso_to_and_from_the_spislave_inst_for_spichain : inout std_logic                    := '0'; --                                                .miso_to_and_from_the_spislave_inst_for_spichain
		spi_slave_to_avalon_mm_master_bridge_0_export_0_sclk_to_the_spislave_inst_for_spichain          : in    std_logic                    := '0'  --                                                .sclk_to_the_spislave_inst_for_spichain
	);
end component i2c_base;


signal i2c_scl_in : std_logic;
signal i2c_sda_in : std_logic;
signal i2c_scl_oe : std_logic;
signal i2c_sda_oe : std_logic;

begin 

	i2c_scl_in <= i2c_scl;
	i2c_sda_in <= i2c_sda;
	
	
	i2c_scl <= '0' when i2c_scl_oe = '1' else 'Z';
	i2c_sda <= '0' when i2c_sda_oe = '1' else 'Z';
	
	
	interface_i2c : i2c_base port map (clk, i2c_sda_in, i2c_scl_in, i2c_sda_oe, i2c_scl_oe, op_led,rstn,mosi,ss,miso,sclk);


end architecture;
	