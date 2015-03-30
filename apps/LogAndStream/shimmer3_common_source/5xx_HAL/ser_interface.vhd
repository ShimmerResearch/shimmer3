----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    10:07:05 11/16/2009 
-- Design Name: 
-- Module Name:    i2cSerialInterface - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.STD_LOGIC_ARITH.ALL;
use IEEE.STD_LOGIC_UNSIGNED.ALL;

entity i2cSerialInterface is
    Port ( clk : in  STD_LOGIC;
	 reset : in  STD_LOGIC;
	 ce : in  STD_LOGIC;
	 sda_in : in std_logic;
	 sda_out : out std_logic;
	 scl : in std_logic;
	 data_in : in std_logic_vector(7 downto 0);
	 data_out : out std_logic_vector(7 downto 0);
	 sda_outEnable : out std_logic;
	 dataValid : out std_logic--;
	-- debugPort : out std_logic_vector(7 downto 0)
	 	 
	 );
end i2cSerialInterface;

architecture Behavioral of i2cSerialInterface is

constant SLAVE_ADDRESS : std_logic_vector(6 downto 0) := "1001101";
signal i2c_address : std_logic_vector(6 downto 0);
signal i2c_data : std_logic_vector(7 downto 0);

signal detect_start,detect_stop : std_logic;

--- REG
signal dataValid_reg : std_logic;
signal Q1_scl,Q2_scl,scl_highTransition,scl_lowTransition : std_logic;

-------------------------------------------------------------
-------- FSM ------------------------------------------------
-------------------------------------------------------------
type state_type is (st1_idle,st2_start,
						  address_receive,address_check,
						  rw_receive,
						  ack,data_ack,
						  data_receive,data_check,
						  
						  data_write,data_written,
						  scl_low,scl_high,
						  stx_stop);

signal state : state_type;

type selector_type is (ADDRESS,RW,ACK,DATA_READ,DATA_WRITE,DATA_ACK);
signal selector : selector_type;

signal bit_counter : integer range 0 to 15 := 0;


begin

-- debounce SCL
process(clk)
begin
   if (clk'event and clk = '1') then
      if (reset = '1') then
         Q1_scl <= '0';
         Q2_scl <= '0';         
      else
         Q1_scl <= scl;
         Q2_scl <= Q1_scl;

      end if;
   end if;
end process;
 
scl_highTransition <= (Q1_scl) and (not Q2_scl);
scl_lowTransition <= (not Q1_scl) and Q2_scl;
-- ************************  START/STOP Detect Process ************************
-- This process detects the start and stop conditions.
-- by using SDA as a clock.
start_det: process(sda_in, reset,state)
begin
        if reset = '1' or state /= st2_start then
                detect_start <= '0';					 
        elsif sda_in'event and sda_in = '0' then
                if scl /= '0' then
                   detect_start <= '1';								
                else
                   detect_start <= '0';
                end if;
        end if;
end process;

stop_det: process(sda_in, reset)
begin
        if reset = '1' then
                detect_stop <= '0';					 
        elsif sda_in'event and sda_in /= '0' then
                if scl /= '0' then
                        detect_stop <= '1';					
                else
                        detect_stop <= '0';
                end if;
        end if;
end process;
-----------------------------------------------------------------
----------------------- FSM -------------------------------------
-----------------------------------------------------------------
SYNC_PROC: process (clk,reset,detect_stop)
	
	variable i2c_reg : std_logic_vector(6 downto 0);
	variable i2c_data_reg : std_logic_vector(7 downto 0);
	variable i2c_cmd : std_logic;
	
   begin
		if reset = '1' then
			state <= st1_idle;
			data_out <= (others => '0');		
      elsif (clk'event and clk = '1') then


				dataValid <= dataValid_reg;					         
				dataValid_reg <= '0';

		case state is
			when st1_idle =>
            sda_out <= '0';
				sda_outEnable <= '0';											

				i2c_address <= (others => '0');
				i2c_data <= (others => '0');				
--				debugPort <= x"00";

				if ce ='1' then
					state <= st2_start;
				end if;

         when st2_start =>
				sda_outEnable <= '0'; -- flag che abilita la scrittura sul bus i2c (linea SDA)		
				bit_counter <= 0;
						
				i2c_reg := (others => '0');
				i2c_data_reg := (others => '0');
--				debugPort <= x"00";				
				if detect_start = '1' then
					state <= address_receive;
				end if;
				
			when address_receive =>
				selector <= ADDRESS;				  -- ricevo per primo l'indirizzo
				if scl_lowTransition = '1' then -- aspetto che il clock SCL sia '0'
					state <= scl_low;
				end if;
			
			when scl_low =>				

				if scl_highTransition = '1' then -- fronto di salita: campiono il dato.					
					case selector is
						when ADDRESS =>					
							i2c_reg := i2c_reg(5 downto 0) & sda_in;		
						when RW =>
							i2c_cmd := sda_in;
						when ACK =>
							sda_out <= '0'; -- NACK		
						when DATA_READ =>
							i2c_data_reg := i2c_data_reg(6 downto 0) & sda_in;
						when DATA_ACK =>
							sda_out <= '0'; -- NACK		
						when DATA_WRITE =>
							sda_out <= data_in(7 - bit_counter);
						when others =>
					end case;
					if  detect_stop = '1' then
						state <= stx_stop;
					else
						state <= scl_high;
					end if;
				end if;

			when scl_high =>		
				
				if scl_lowTransition = '1' then
					state <= scl_low;
					
					case selector is
						when ADDRESS =>
							bit_counter <= bit_counter + 1;
							if bit_counter = 6 then -- ho ricevuto tutto l'indirizzo, controllo 
								state <= address_check;
							end if;		
						when RW => 
								state <= rw_receive;
						when ACK =>
							if i2c_cmd = '0' then -- WRITE TO SLAVE (PIC -> FPGA)
								state <= data_receive;
							else
								state <= data_write;
							end if;
						when DATA_ACK =>
							if i2c_cmd = '0' then -- WRITE TO SLAVE (PIC -> FPGA)
								state <= data_receive;
							else
							-- bisogna gestire il caso in cui ci siano altri dati da inviare!
								state <= data_write;
							end if;
						
						when DATA_READ =>
							bit_counter <= bit_counter + 1;
							if bit_counter = 7 then -- ho ricevuto tutto il dato
								state <= data_check;
							end if;		
						when DATA_WRITE =>
							bit_counter <= bit_counter + 1;
							if bit_counter = 7 then -- ho ricevuto tutto il dato
								state <= data_written;
							end if;		
						
						when others =>
					end case;	
				
					if  detect_stop = '1' then
						state <= stx_stop;
					end if;

				end if;
		

			when address_check =>
				if i2c_address = SLAVE_ADDRESS then
					selector <= RW;
					state <= scl_low;
				else
					state <= st2_start;
				end if;

			when rw_receive =>
				selector <= ACK;
				state <= scl_low;
				sda_outEnable <= '1'; -- Abilito la porta in uscita
			
			when data_receive =>
				-- ACK inviato, riporto la linea in modalita' input				
				sda_outEnable <= '0';
				selector <= DATA_READ;
				state <= scl_low;				
				bit_counter <= 0;

			when data_check =>
				selector <= DATA_ACK;
				state <= scl_low;
--				debugPort <=  i2c_data;--i2c_address & i2c_cmd;
				dataValid_reg <= '1';
				data_out <= i2c_data;
				sda_outEnable <= '1';

			when data_write =>
				-- ACK inviato, riporto la linea in modalita' input				
				sda_outEnable <= '1';
				selector <= DATA_WRITE;
				state <= scl_low;				
				bit_counter <= 0;
			when data_written =>
				sda_outEnable <= '0';
			
			when stx_stop =>
				
			
         when others =>

		end case;
		
	end if;
	
	i2c_address <= i2c_reg;
   i2c_data <= i2c_data_reg;

   end process SYNC_PROC;
 


end Behavioral;

