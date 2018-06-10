import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'
    
    # load pre trained NN (frozen graph)
    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path);
    # define a graph
    glob_graph = tf.get_default_graph();
    # get weights from layers 0, 3, 4, and 7 + the dropout layer 
    input_layer_weight = glob_graph.get_tensor_by_name(vgg_input_tensor_name);
    keep_prob = glob_graph.get_tensor_by_name(vgg_keep_prob_tensor_name);
    layer3_weight = glob_graph.get_tensor_by_name(vgg_layer3_out_tensor_name);
    layer4_weight = glob_graph.get_tensor_by_name(vgg_layer4_out_tensor_name);
    layer7_weight = glob_graph.get_tensor_by_name(vgg_layer7_out_tensor_name);
    
    return input_layer_weight, keep_prob, layer3_weight, layer4_weight, layer7_weight

tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer3_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer7_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
        
    # 1x1 convolution of layer 7 in frozen graph
    conv_1x1_layer7 = tf.layers.conv2d(vgg_layer7_out, 
                                       num_classes, 
                                       1, 
                                       padding='same', 
                                       kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));
    
    # deconvolution of layer 7 1x1 output
    deconvolution_output_layer7 = tf.layers.conv2d_transpose(conv_1x1_layer7, 
                                                            num_classes, 
                                                            4,
                                                            strides = (2,2), 
                                                            padding='same', 
                                                            kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));
    
    
    # 1x1 convolution of layer 4 in frozen graph
    conv_1x1_layer4 = tf.layers.conv2d(vgg_layer4_out, 
                                       num_classes, 
                                       1, 
                                       padding='same', 
                                       kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));
    # skip connection of layer 7 deconvolution and layer 4 1x1 convolution
    skip_layer4_layer7 = tf.add(deconvolution_output_layer7, conv_1x1_layer4);
    
    # deconvolution of skip connection
    deconvolution_skip_layer = tf.layers.conv2d_transpose(skip_layer4_layer7, 
                                                           num_classes, 
                                                           4,
                                                           strides = (2,2), 
                                                           padding='same', 
                                                           kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));
   
    # 1x1 convolution of layer 3 in frozen graph                                                                
    conv_1x1_layer3 = tf.layers.conv2d(vgg_layer3_out, 
                                       num_classes, 
                                       1, 
                                       padding='same', 
                                       kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));
    
    # skip connection deconvolution last skip and layer 4 1x1 convolution
    skip_deconvolution_skip_layer_conv_1x1_layer3 = tf.add(deconvolution_skip_layer, conv_1x1_layer3);
    
    #deconvolution of deconvoluted skip layer and deconvoluted layer 3
    final_deconvolution = tf.layers.conv2d_transpose(skip_deconvolution_skip_layer_conv_1x1_layer3, 
                                                     num_classes, 
                                                     16,
                                                     strides = (8,8), 
                                                     padding='same', 
                                                     kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3));

    return final_deconvolution


tests.test_layers(layers)


def optimize(nn_last_layer, correct_label, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # reduce dimension
    logits_arg = tf.reshape(nn_last_layer, (-1, num_classes));
    labels_arg = tf.reshape(correct_label, (-1, num_classes));
    
    # define cross entropy loss based on logits and labels
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits_arg, labels=labels_arg))
    # define the optimizer
    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    # train
    training_operation = optimizer.minimize(cross_entropy_loss)
    
    return logits_arg, training_operation, cross_entropy_loss

tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    # initialize variables prior to training
    sess.run(tf.global_variables_initializer())
    # train
    
    print("#############")
    print("Training started ... ")
    for epoch in range(epochs):
        for image, label in get_batches_fn(batch_size):
            _, loss = sess.run([train_op, cross_entropy_loss], 
                               feed_dict = {input_image: image, correct_label: label, keep_prob: 0.5, learning_rate: 0.000002 })
            print(loss) 
        print("Run no. ")
        print(epoch)
        print("\n")
tests.test_train_nn(train_nn)


def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')
        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # load the parts of the frozen graph 
        in_img, keep_prob_droput, w_layer3, w_layer4, w_layer7 = load_vgg(sess, vgg_path);
        # get the FCN
        deconvoluted_input = layers(w_layer3, w_layer4, w_layer7, num_classes);
        # define a variable for the label
        correct_label = tf.placeholder(tf.int32, shape = (None, None, None, num_classes));
        # define a variable for the learning rate
        learning_rate = tf.placeholder(tf.float32);
        # start optimizer
        logits, training_operation, cross_entropy_loss = optimize(deconvoluted_input, correct_label, learning_rate, num_classes);
        #define batch and epochs
        epochs = 36;
        batch_size = 6;
        #train model
        train_nn(sess, 
                 epochs, 
                 batch_size, 
                 get_batches_fn, 
                 training_operation, 
                 cross_entropy_loss, 
                 in_img, 
                 correct_label, 
                 keep_prob_droput, 
                 learning_rate);
        
        # safe model
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, keep_prob_droput, in_img);
        
        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
