library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart_rx is
    generic (
        g_CLKS_PER_BIT : integer := 2083
    );
    port (
        i_clk   : in  std_logic; 
        i_rst   : in  std_logic; 
        i_rxd   : in  std_logic; 
        o_rx_data       : out std_logic_vector(7 downto 0);
        o_rx_data_valid : out std_logic 
    );
end entity uart_rx;

architecture rtl of uart_rx is

    type t_fsm_state is (
        s_idle,        
        s_start_bit,   
        s_rx_data,     
        s_stop_bit,    
        s_cleanup      
    );

    signal r_fsm_state : t_fsm_state := s_idle;
    signal r_clk_counter : integer range 0 to g_CLKS_PER_BIT := 0;
    signal r_bit_counter : integer range 0 to 7 := 0;
    signal r_rx_data_reg : std_logic_vector(7 downto 0);
    signal r_rxd_sync : std_logic_vector(1 downto 0);

begin
    p_uart_rx : process (i_clk)
        variable v_rxd_synced : std_logic;
    begin
        if (rising_edge(i_clk)) then
        
            r_rxd_sync(0) <= i_rxd;
            r_rxd_sync(1) <= r_rxd_sync(0);
            v_rxd_synced  := r_rxd_sync(1); 

            if (i_rst = '1') then
                r_fsm_state     <= s_idle;
                r_clk_counter   <= 0;
                r_bit_counter   <= 0;
                o_rx_data_valid <= '0';
                
            else
                o_rx_data_valid <= '0';

                case r_fsm_state is
                    when s_idle =>
                        if (v_rxd_synced = '0') then
                            r_fsm_state   <= s_start_bit;
                            r_clk_counter <= 0;
                        end if;

                    when s_start_bit =>
                        if (r_clk_counter = (g_CLKS_PER_BIT / 2)) then
                            if (v_rxd_synced = '0') then
                                r_fsm_state   <= s_rx_data;
                                r_clk_counter <= 0;
                                r_bit_counter <= 0;
                            else
                                r_fsm_state <= s_idle;
                            end if;
                        else
                            r_clk_counter <= r_clk_counter + 1;
                        end if;

                    when s_rx_data =>
                        if (r_clk_counter = (g_CLKS_PER_BIT - 1)) then
                            r_clk_counter <= 0;
                            r_rx_data_reg(r_bit_counter) <= v_rxd_synced;
                            if (r_bit_counter = 7) then
                                r_fsm_state <= s_stop_bit;
                            else
                                r_bit_counter <= r_bit_counter + 1;
                            end if;
                        else
                            r_clk_counter <= r_clk_counter + 1;
                        end if;

                    when s_stop_bit =>
                        if (r_clk_counter = (g_CLKS_PER_BIT - 1)) then
                            r_fsm_state <= s_cleanup;
                        else
                            r_clk_counter <= r_clk_counter + 1;
                        end if;

                    when s_cleanup =>
                        o_rx_data       <= r_rx_data_reg;
                        o_rx_data_valid <= '1'; 
                        r_fsm_state     <= s_idle;

                    when others =>
                        r_fsm_state <= s_idle;

                end case;
            end if;
        end if;
    end process p_uart_rx;

end architecture rtl;