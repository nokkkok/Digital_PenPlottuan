library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- This module instantiates the uart_rx and builds the 9-byte packet
entity packet_parser is
    generic (
        g_CLKS_PER_BIT : integer := 2083 -- 20,000,000 / 9600
    );
    port (
        -- System signals
        i_clk   : in  std_logic;
        i_rst   : in  std_logic;
        
        -- UART input
        i_rxd   : in  std_logic;
        
        -- Outputs to top level
        o_x_steps       : out std_logic_vector(23 downto 0);
        o_y_steps       : out std_logic_vector(23 downto 0);
        o_control       : out std_logic_vector(7 downto 0);
        o_new_command_valid : out std_logic;
        
        -- *** NEW PORT ADDED HERE ***
        o_rx_activity_led : out std_logic
    );
end entity packet_parser;

architecture rtl of packet_parser is

    -- FSM state definitions
    type t_fsm_state is (
        s_idle,          -- Wait for Start Byte (0xFE)
        s_get_x_h,       -- Get X-Steps High
        s_get_x_m,       -- Get X-Steps Mid
        s_get_x_l,       -- Get X-Steps Low
        s_get_y_h,       -- Get Y-Steps High
        s_get_y_m,       -- Get Y-Steps Mid
        s_get_y_l,       -- Get Y-Steps Low
        s_get_control,   -- Get Control Byte
        s_get_end        -- Get End Byte (0xFF)
    );
    
    signal r_fsm_state : t_fsm_state := s_idle;

    -- Wires from the uart_rx module
    signal w_rx_data        : std_logic_vector(7 downto 0);
    signal w_rx_data_valid  : std_logic;
    
    -- Temporary registers to build the packet
    signal r_temp_x_steps : std_logic_vector(23 downto 0);
    signal r_temp_y_steps : std_logic_vector(23 downto 0);
    signal r_temp_control : std_logic_vector(7 downto 0);
    
    -- Registered versions of the outputs
    signal r_x_steps       : std_logic_vector(23 downto 0) := (others => '0');
    signal r_y_steps       : std_logic_vector(23 downto 0) := (others => '0');
    signal r_control       : std_logic_vector(7 downto 0)  := (others => '0');
    signal r_new_cmd_valid : std_logic := '0';

    -- *** NEW SIGNAL FOR RX ACTIVITY LED ***
    -- Counter for ~0.05s (2^20 cycles @ 20MHz)
    signal r_activity_counter : unsigned(19 downto 0) := (others => '0');

    -- Component for the uart_rx
    component uart_rx is
        generic (
            g_CLKS_PER_BIT : integer := 2083
        );
        port (
            i_clk           : in  std_logic;
            i_rst           : in  std_logic;
            i_rxd           : in  std_logic;
            o_rx_data       : out std_logic_vector(7 downto 0);
            o_rx_data_valid : out std_logic
        );
    end component uart_rx;

begin

    -- Instantiate the uart_rx module
    u_uart_rx : uart_rx
        generic map (
            g_CLKS_PER_BIT => g_CLKS_PER_BIT
        )
        port map (
            i_clk           => i_clk,
            i_rst           => i_rst,
            i_rxd           => i_rxd,
            o_rx_data       => w_rx_data,
            o_rx_data_valid => w_rx_data_valid
        );


    -- FSM process to parse the packet
    p_packet_parser : process (i_clk)
    begin
        if (rising_edge(i_clk)) then
            if (i_rst = '1') then
                r_fsm_state     <= s_idle;
                r_new_cmd_valid <= '0';
                r_temp_x_steps  <= (others => '0');
                r_temp_y_steps  <= (others => '0');
                r_temp_control  <= (others => '0');
                r_x_steps       <= (others => '0');
                r_y_steps       <= (others => '0');
                r_control       <= (others => '0');
                r_activity_counter <= (others => '0'); -- Reset counter
            else
                r_new_cmd_valid <= '0';
                
                -- *** RX ACTIVITY LED LOGIC ***
                if (w_rx_data_valid = '1') then
                    r_activity_counter <= (others => '1'); -- Load counter
                elsif (r_activity_counter /= 0) then
                    r_activity_counter <= r_activity_counter - 1; -- Decrement
                end if;
                
                -- This FSM runs only when the uart_rx module gives us a new byte
                if (w_rx_data_valid = '1') then
                    case r_fsm_state is
                        when s_idle =>
                            if (w_rx_data = x"FE") then
                                r_fsm_state <= s_get_x_h;
                            else
                                r_fsm_state <= s_idle;
                            end if;
                            
                        when s_get_x_h =>
                            r_temp_x_steps(23 downto 16) <= w_rx_data;
                            r_fsm_state <= s_get_x_m;
                            
                        when s_get_x_m =>
                            r_temp_x_steps(15 downto 8) <= w_rx_data;
                            r_fsm_state <= s_get_x_l;
                            
                        when s_get_x_l =>
                            r_temp_x_steps(7 downto 0) <= w_rx_data;
                            r_fsm_state <= s_get_y_h;
                            
                        when s_get_y_h =>
                            r_temp_y_steps(23 downto 16) <= w_rx_data;
                            r_fsm_state <= s_get_y_m;
                            
                        when s_get_y_m =>
                            r_temp_y_steps(15 downto 8) <= w_rx_data;
                            r_fsm_state <= s_get_y_l;
                            
                        when s_get_y_l =>
                            r_temp_y_steps(7 downto 0) <= w_rx_data;
                            r_fsm_state <= s_get_control;
                            
                        when s_get_control =>
                            r_temp_control <= w_rx_data;
                            r_fsm_state <= s_get_end;
                            
                        when s_get_end =>
                            if (w_rx_data = x"FF") then
                                -- VALID PACKET
                                r_x_steps       <= r_temp_x_steps;
                                r_y_steps       <= r_temp_y_steps;
                                r_control       <= r_temp_control;
                                r_new_cmd_valid <= '1'; -- Pulse valid flag
                            end if;
                            r_fsm_state <= s_idle;
                            
                        when others =>
                            r_fsm_state <= s_idle;
                    end case;
                end if;
            end if;
        end if;
    end process p_packet_parser;

    -- Connect registered signals to the final output ports
    o_x_steps           <= r_x_steps;
    o_y_steps           <= r_y_steps;
    o_control           <= r_control;
    o_new_command_valid <= r_new_cmd_valid;

    -- *** ASSIGN ACTIVITY LED (Active-High) ***
    o_rx_activity_led <= '1' when r_activity_counter /= 0 else '0';

end architecture rtl;
