/*
 * Amazon FPGA Hardware Development Kit
 *
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Amazon Software License (the "License"). You may not use
 * this file except in compliance with the License. A copy of the License is
 * located at
 *
 *    http://aws.amazon.com/asl/
 *
 * or in the "license" file accompanying this file. This file is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, express or
 * implied. See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

// C++ library
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

extern "C"{
  #include "test_dram_dma_common.h"
   #include "fpga_pci.h"
   #include "fpga_mgmt.h"
   #include "fpga_dma.h"
   #include "utils/lcd.h"

}


#define	MEM_16G              (1ULL << 34)
#define USER_INTERRUPTS_MAX  (16)

using namespace std;
namespace py = pybind11;

/* use the standard out logger */
static const struct logger *logger = &logger_stdout;

void usage(const char* program_name);
int dma_example(int slot_id, size_t buffer_size);

py::array_t<float> hjsolver_test(vector<vector<float>> obstacle_positions, vector<float> radius);
float int_to_float(int a);
int float_to_int(float a);
void my_linspace(float lb, float mb, float A[], int dim);
void Initialize_V(int* A, vector<vector<float>> obstacle_positions, vector<float> radius);
int add(int i, int j);

pci_bar_handle_t pci_bar_handle = PCI_BAR_HANDLE_INIT;
int pf_id = 0;
int bar_id = 0;
int fpga_attach_flags = 0;
int slot_id = 0;

/*
int main(int argc, char **argv) {
    int rc;
    int slot_id = 0;

    switch (argc) {
    case 1:
        break;
    case 3:
        sscanf(argv[2], "%x", &slot_id);
        break;
    default:
        usage(argv[0]);
        return 1;
    }

    // setup logging to print to stdout
    rc = log_init("test_dram_dma");
    fail_on(rc, out, "Unable to initialize the log.");
    rc = log_attach(logger, NULL, 0);
    cout << rc << endl;
    fail_on(rc, out, "%s", "Unable to attach to the log.");

    //initialize the fpga_plat library 
    rc = fpga_mgmt_init();
    fail_on(rc, out, "Unable to initialize the fpga_mgmt library");

    // check that the AFI is loaded 
    log_info("Checking to see if the right AFI is loaded...");
#ifndef SV_TEST
    //cout << "I'm here\n";
    rc = check_slot_config(slot_id);
    fail_on(rc, out, "slot config is not correct");
#endif

    //run my own example 
    //rc = dma_example(slot_id, 1000);
    rc = hjsolver_test(slot_id);
    cout << "pass or not " << rc << endl;
    //fail_on(rc, out, "Dandelion example failed\n");

out:
    log_info("TEST %s", (rc == 0) ? "PASSED" : "FAILED");
    return rc;
}*/

void usage(const char* program_name) {
  printf("usage: %s [--slot <slot>]\n", program_name);
}

int dma_example(int slot_id, size_t buffer_size) {
  int write_fd, read_fd, dimm, rc;

  write_fd = -1;
  read_fd = -1;

  uint8_t *write_buffer = (uint8_t *) malloc(buffer_size);
  uint8_t *read_buffer = (uint8_t* )malloc(buffer_size);
  if (write_buffer == NULL || read_buffer == NULL) {
    rc = -ENOMEM;
    goto out;
  }

  read_fd = fpga_dma_open_queue(FPGA_DMA_XDMA, slot_id,
				/*channel*/ 0, /*is_read*/ true);
  cout << "read_fd " << read_fd << endl;
  fail_on((rc = (read_fd < 0) ? -1 : 0), out, "unable to open read dma queue");

  write_fd = fpga_dma_open_queue(FPGA_DMA_XDMA, slot_id,
        /*channel*/ 0, /*is_read*/ false);
  cout << "write_fd " << write_fd << endl;
  fail_on((rc = (write_fd < 0) ? -1 : 0), out, "unable to open write dma queue");

  rc = fill_buffer_ones(write_buffer, buffer_size);
  fail_on(rc, out, "unabled to initialize buffer");

  for (dimm = 0; dimm < 1; dimm++) {
    rc = fpga_dma_burst_write(write_fd, write_buffer, buffer_size,
			      dimm * MEM_16G);
    cout << "rc dma_burst_write " << rc << endl;
    fail_on(rc, out, "DMA write failed on DIMM: %d", dimm);
  }

  bool passed;
  passed = true;
  for (dimm = 0; dimm < 1; dimm++) {
    rc = fpga_dma_burst_read(read_fd, read_buffer, buffer_size,
			     dimm * MEM_16G);
    fail_on(rc, out, "DMA read failed on DIMM: %d", dimm);
        cout << "rc dma_burst_read " << rc << endl;

    uint64_t differ = buffer_compare(read_buffer, write_buffer, buffer_size);
    if (differ != 0) {
      cout << "DIMM failled\n";
      log_error("DIMM %d failed with %lu bytes which differ", dimm, differ);
      passed = false;
    } else {
      cout << "DIMM " << dimm << "passed!\n";
      log_info("DIMM %d passed!", dimm);
    }
  }
  rc = (passed) ? 0 : 1;

out:
  if (write_buffer != NULL) {
    free(write_buffer);
  }
  if (read_buffer != NULL) {
    free(read_buffer);
  }
  if (write_fd >= 0) {
    close(write_fd);
  }
  if (read_fd >= 0) {
    close(read_fd);
  }
  // if there is an error code, exit with status 1
  return (rc != 0 ? 1 : 0);
}

py::array_t<float> hjsolver_test(vector<vector<float>> obstacle_positions, vector<float> radius) {
    int rc_control;
    int rc_memory;

    uint32_t in_hi_addr, in_lo_addr;
    uint32_t out_hi_addr, out_lo_addr;
    uint32_t cycle_count = 0;
    uint32_t control_reg = 0;

    //Variables for read/write data through DMA
    int write_fd, read_fd;
    read_fd = -1;
    write_fd = -1;
    
    // Test input
    int buffer_size;
    buffer_size= 60 * 60 * 20 * 36;
    int *write_buffer;
    write_buffer = (int *) malloc(buffer_size * sizeof(uint32_t));    

    Initialize_V((int *)write_buffer, obstacle_positions, radius);
    // Print 60 values
    //for(int i = 0; i < 60; i++){ // along x direction
    //  cout << write_buffer([i*60*20*36 + 30*20*36]) << " " ;
    //}
    //cout << endl;


    int *read_buffer;
    read_buffer= (int *) malloc(buffer_size * sizeof(uint32_t));
    
    if (write_buffer == NULL || read_buffer == NULL) {
        rc_memory = -ENOMEM;
        //goto out;
    }
    // Write to DMA buffer
    //rc_memory = fill_buffer_urandom(write_buffer, buffer_size);

    // Open the channel
    read_fd = fpga_dma_open_queue(FPGA_DMA_XDMA, slot_id,
        /*channel*/ 0, /*is_read*/ true);
    //fail_on((rc_memory = (read_fd < 0) ? -1 : 0), out, "unable to open read dma queue");

    write_fd = fpga_dma_open_queue(FPGA_DMA_XDMA, slot_id,
        /*channel*/ 0, /*is_read*/ false);
    //fail_on((rc_memory = (write_fd < 0) ? -1 : 0), out, "unable to open write dma queue");
    //fail_on(rc_memory, out, "unabled to initialize buffer");

    // Write to dram through DMA
    rc_memory = fpga_dma_burst_write(write_fd, (uint8_t *) write_buffer, buffer_size * sizeof(int), 0);
    //fail_on(rc_memory, out, "DMA write failed");

    // write at the output address
    rc_memory = fill_buffer_zeros((uint8_t *) read_buffer, buffer_size * sizeof(int));

    rc_memory = fpga_dma_burst_write(write_fd, (uint8_t *) read_buffer, buffer_size * sizeof(int), 0x10000000);
    //fail_on(rc_memory, out, "DMA write failed");

    log_info("Starting AXI Master to DDR test");
    cout << "Starting AXI Master to DDR test\n";
    /* Initializing control registers
     * Register File.
     * Six 32-bit register file.
     *
     * -------------------------------
     * Register description    | addr
     * ------------------------|-----
     * Control status register | 0x500
     * Ecnt1                   | 0x504
     * Ecnt2                   | 0x508
     * Input addr lsb          | 0x50c
     * Input addr msb          | 0x510
     * Output addr lsb         | 0x514
     * Output addr msb         | 0x518
     * -------------------------------
     *
     * ------------------------------
     * Control status register | bit
     * ------------------------------
     * Launch                  | 0
     * Finish                  | 1
     * ------------------------------
    */

    static uint64_t ccr_control  = 0x500;
    static uint64_t ccr_cycle    = 0x504;
    //static uint64_t ccr_ptime    = 0x508;
    static uint64_t ccr_in_lsb   = 0x50C;
    static uint64_t ccr_in_msb   = 0x510;
    static uint64_t ccr_out_lsb  = 0x514;
    static uint64_t ccr_out_msb  = 0x518;


    in_hi_addr = 0x00000000;
    in_lo_addr = 0x00000000;

    out_hi_addr = 0x00000000;
    out_lo_addr = 0x10000000;

    /* write a value into the mapped address space */
    cout <<"Initializing dandelion accelerator:\n";

//printf("Writing 0x%08x to Dandelion in lsb register (0x%016lx)\n\n", in_lo_addr, ccr_in_lsb); 
    cout << "writing to lsb register";
    rc_control = fpga_pci_poke(pci_bar_handle, ccr_in_lsb, in_lo_addr);
    //fail_on(rc_control, out, "Unable to write to the fpga !");

//printf("Writing 0x%08x to Dandelion in msb register (0x%016lx)\n\n", in_hi_addr, ccr_in_msb);
    rc_control = fpga_pci_poke(pci_bar_handle, ccr_in_msb, in_hi_addr);
    //fail_on(rc_control, out, "Unable to write to the fpga !");

//printf("Writing 0x%08x to Dandelion out lsb register (0x%016lx)\n\n", out_lo_addr, ccr_out_lsb);
    rc_control = fpga_pci_poke(pci_bar_handle, ccr_out_lsb, out_lo_addr);
    //fail_on(rc_control, out, "Unable to write to the fpga !");


//printf("Writing 0x%08x to Dandelion out msb register (0x%016lx)\n\n", out_hi_addr, ccr_out_msb);
    rc_control = fpga_pci_poke(pci_bar_handle, ccr_out_msb, out_hi_addr);
    //fail_on(rc_control, out, "Unable to write to the fpga !");

//printf("Launching -- Writing 0x1 to dandelion ctrl register:\n");
    cout << "Launching -- Writing 0x1 to dandelion ctrl register:\n";
    rc_control = fpga_pci_poke(pci_bar_handle, ccr_control, 0x1);
    //fail_on(rc_control, out, "Unable to write to the fpga !");

    clock_t begin ;
    begin = clock(); 
    do{
        rc_control = fpga_pci_peek(pci_bar_handle, ccr_control, &control_reg);
        //fail_on(rc_control, out, "Unable to read read from the fpga !");
	} while(control_reg != 2);
  
    clock_t diff;
    diff = clock() - begin;
    double time_spent;
    time_spent= (double ) diff/ CLOCKS_PER_SEC;

    rc_control = fpga_pci_peek(pci_bar_handle, ccr_cycle, &cycle_count);
    //fail_on(rc_memory, out, "Unable to read read from the fpga !");
    cout << "Execution finished in [ %d ] cycle\n" <<  cycle_count << endl;

    // rc = axi_mstr_ddr_access(slot_id, pci_bar_handle, ddr_hi_addr, ddr_lo_addr, ddr_data);
    // fail_on(rc, out, "Unable to access DDR A.");
    
    cout << "time taken " <<  time_spent << endl;

    // Read from dram
    rc_memory = fpga_dma_burst_read(read_fd, (uint8_t *) read_buffer, buffer_size * sizeof(int), 0x10000000);
    //fail_on(rc_memory, out, "DMA read failed on reading form out buffer.");
  
    bool flag;
    flag = false;
    int count_nz;
    count_nz= 0;

    //py::array_t<float> result;
    auto result = py::array_t<float>(buffer_size);
    py::buffer_info buf = result.request();
    float *ptr1 = static_cast<float *>(buf.ptr);

    for(uint32_t i = 0; i < buffer_size; i++){
      //printf("Buffer [%d]: %d\n", i, read_buffer[i]);
      if(read_buffer[i] <= 0) {
	flag = true;
	count_nz++;
      }
      
      ptr1[i] = int_to_float(read_buffer[i]);
      //float val_in_f = int_to_float(read_buffer[i]);
      //if(read_buffer[i] != write_buffer[i])
      //count_nz++;
      //printf("%lf ", val_in_f);
    }
    if(!flag) cout << "sth might worng\n";
    cout << "count_nz is " << count_nz << endl;

out:
    if (write_buffer != NULL) {
        free(write_buffer);
    }
    if (read_buffer != NULL) {
        free(read_buffer);
    }
    if (write_fd >= 0) {
        close(write_fd);
    }
    if (read_fd >= 0) {
        close(read_fd);
    }
    // if there is an error code, exit with status 1
    return result;
    //return (rc_control != 0 ? (rc_memory != 0 ? 1 : 0) : 0);

}


float int_to_float(int a){
  int mantissa = 27;
  int multiplier = pow(2, mantissa);
  
  float result = (float) a / multiplier;

  return result;
}

int float_to_int(float a){
  int mantissa = 27;
  int result = (int) round(a * pow(2, mantissa));
  return result;
}

void my_linspace(float lb, float mb, float A[], int dim){
  float dx;
  dx  = (mb - lb) / (dim - 1);

  for(int i = 0; i < dim; i++){
    A[i] = dx * i + lb;
  }

  return;

}

void Initialize_V(int* A, vector<vector<float>> obst, vector<float> radius){
  float x_lb = -3.0, x_mb = 3.0;
  int y_lb = -1.0, y_mb = 4.0;
  int v_lb = 0.0, v_mb = 4.0;
  int t_lb = -M_PI, t_mb = M_PI;

  int x_dim = 60, y_dim = 60, v_dim = 20, t_dim = 36;
  
  float x_arr[60], y_arr[60], v_arr[20], t_arr[36];


  //  float radius = 0.75;
  my_linspace(x_lb, x_mb, x_arr, x_dim);
  my_linspace(y_lb, y_mb, y_arr, y_dim);
  my_linspace(v_lb, v_mb, v_arr, v_dim);
  my_linspace(t_lb, t_mb, t_arr, t_dim);

  //float x_list[3] = {1.4, 0.0, -1.9};
  //float y_list[3] = {2.15, 1.0, 1.55};
  //float r_list[3] = {0.7, 0.7, 0.7};
  int count_nz = 0;

  for(int i = 0; i < x_dim; i++){
    for(int j = 0; j < y_dim; j++){
      for(int k = 0; k < v_dim; k++){
	for(int l = 0; l < t_dim; l++){
	  float min_val = 1000;
	  for(int m = 0; m < radius.size(); m++){
	    float val_func = sqrt((y_arr[j] - obst[m][1]) * (y_arr[j] - obst[m][1]) + (x_arr[i] - obst[m][0]) * (x_arr[i] - obst[m][0])) 
	      - radius[m];
	    min_val = min(val_func, min_val);
	  }
	  //int convert_val = float_to_int(sqrt(( y_arr[j] -1.0 ) * (y_arr[j] - 1.0) + x_arr[i]*x_arr[i]) - radius);
	  int convert_val = float_to_int(min_val);
	  A[i * y_dim * v_dim * t_dim + j * v_dim * t_dim + k * t_dim + l] = convert_val;
	  if(min_val < 0) count_nz++;
	}
      }
    }
  }

  //printf("count_nz before: %d\n", count_nz);
  cout << "count_nz before " << count_nz << endl;
}

int add(int i, int j) {
  return i + j;
}

void InitializeFPGA(){
  cout << "Initializing FPGA ...\n";
  int rc;
  int slot_id = 0;

  //initialize the fpga_plat library 
  rc = fpga_mgmt_init();
  if(rc != 0) cout << "Cannot initialize the fpga platform\n";

  // check that the AFI is loaded 
  cout << "Checking to see if the right AFI is loaded ...\n";

  rc = check_slot_config(slot_id);
  if(rc != 0) cout << "slot config is not correct\n";

  rc = fpga_pci_attach(slot_id, pf_id, bar_id, fpga_attach_flags, &pci_bar_handle);
  if(rc != 0) cout << "Unable to attach to the AFI on slot id " <<  slot_id << endl;
   //fail_on(rc_control, out, "Unable to attach to the AFI on slot id %d", slot_id);

  //log_info("TEST %s", (rc == 0) ? "PASSED" : "FAILED");
  if(rc == 0) cout << "PASSED\n";
  else cout << "FAILED\n";
  return ;
}

PYBIND11_MODULE(newexample, m) {
  m.doc() = "HJ solver -- Python interface"; // optional module docstring

  m.def("add", &add, "A function which adds two numbers");
  m.def("InitializeFPGA", &InitializeFPGA);
  m.def("hjsolver_test", &hjsolver_test, py::arg("obstacle_positions"), py::arg("radius"));
  //hjsolver_test()
}
